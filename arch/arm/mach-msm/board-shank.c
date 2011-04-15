/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/cpufreq.h>
#include <linux/io.h>
#include <linux/usb/mass_storage_function.h>
#include <linux/spi/spi.h>
#include <linux/gpio_keys_pe.h>
#include <linux/mfd/pmic8058.h>
#include <linux/pwm.h>
#include <linux/hsuart.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/pwm.h>
#include <linux/mutex.h>
#include <linux/pinmux.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/gpio.h>
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
#include <linux/input/pmic8058-keypad.h>
#include <mach/msm_ts.h>
#include <mach/pmic.h>
#include <mach/qdsp5v2/aux_pcm.h>
#include <linux/cy8ctma300.h>
#include <mach/rpc_server_handset.h>
#include <mach/rpc_pmapp.h>

#include <asm/mach/mmc.h>
#include <mach/vreg.h>
#include "devices.h"
#include "timer.h"
#include "socinfo.h"
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android.h>
#endif
#include "pm.h"
#include "spm.h"
#include <linux/msm_kgsl.h>
#include <mach/dal_axi.h>

#include "proc_comm.h"
#include <mach/msm_fb.h>

#ifdef CONFIG_LEDS_LP8501
#include <linux/i2c_lp8501_led.h>
#endif

#ifdef CONFIG_BLUETOOTH_POWER_STATE
#include <linux/bluetooth-power-pe.h>
#endif

#ifdef CONFIG_USER_PINS
#include <linux/user-pins.h>
#endif

#ifdef CONFIG_HRES_COUNTER
#include <linux/hres_counter.h>
#endif

#ifdef CONFIG_A6
#include <linux/a6_sbw_interface.h>
#include <linux/a6.h>
#include "a6_sbw_impl_shank.h"
#endif

#include "board-shank-gpios.h"

#ifdef CONFIG_MSM_VIBRATOR
#include <linux/vibrator.h>
#endif

#ifdef CONFIG_NDUID
#include <linux/nduid.h>
#endif

enum broadway_board_types {
	BW_PROTO = 0,
	BW_EVT1,
	BW_EMU2,
	BW_EVT2,
	BW_EVT3,
	BW_DVT1,
	BW_DVT2,
	BW_DVT3,
	BW_PVT,
};

static u32 board_type = 0;

enum wlan_reset_pin_owner {
	WLAN_NONE      = 0,
	WLAN_BT        = 1,
	WLAN_WIFI      = 1<<1,
};

// Frame buffer size: each buffer is 320x400x32bpp=512K
// // // We have three of these -> total: 1.5M
#define MSM_FB0_SIZE          0x180000 
#define MSM_FB1_SIZE          0x180000
#define MSM_PMEM_GPU1_SIZE      0x1000000
#define MSM_GPU_PHYS_SIZE       SZ_2M
#define MSM_PMEM_ADSP_SIZE      0x2000000
#define PMEM_KERNEL_EBI1_SIZE   0x600000

#define PMIC_GPIO_INT		27
#define PMIC_VREG_WLAN_LEVEL	2900


/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

int pm8058_gpios_init(struct pm8058_chip *pm_chip)
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

	if (machine_is_shank()) {
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
        { 33, 24,	      0xFF }, // q  [32]
        { 34, 25, 32,         0xFF }, // w  [33]
        { 35, 26, 33,         0xFF }, // e  [34]
        { 36, 27, 34,         0xFF }, // r  [35]
        { 37, 28, 35,         0xFF }, // t  [36]
        { 38, 29, 36,         0xFF }, // y  [37]
        {  0, 30, 37,         0xFF }, // u  [38]
};

static const unsigned int broadway_keymap[] = {
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
		.start	= PM8058_IRQ_KEYPAD,
		.end	= PM8058_IRQ_KEYPAD,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= PM8058_IRQ_KEYSTUCK,
		.end	= PM8058_IRQ_KEYSTUCK,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct pmic8058_keypad_data broadway_keypad_data = {
	.input_name		= "broadway-keypad",
	.input_phys_device	= "broadway-keypad/input0",
	.num_rows		= 5,
	.num_cols		= 7,
	.rows_gpio_start	= 8,
	.cols_gpio_start	= 0,
	.keymap_size		= ARRAY_SIZE(broadway_keymap),
	.keymap			= broadway_keymap,
	.debounce_ms		= {5, 10},
	.scan_delay_ms		= 32,
	.row_hold_ns		= 91500,
	.wakeup			= 1,
	.key_prox_timeout 	= 50,
	.key_prox_width 	= 6,
	.rep_delay		= 500,
	.rep_period		= 100,
	.key_prox_map   	= board_key_prox_map,
};

#ifdef CONFIG_KEYBOARD_GPIO_PE
static struct gpio_keys_button shank_gpio_keys_buttons[] = {
	[0] = {
		.code			= KEY_VOLUMEUP,
		.gpio			= GPIO_VOL_UP,
		.active_low		= 1,
		.desc			= "volume up",
		.debounce		= 10,
		.type			= EV_KEY,
		.wakeup			= 1,
#ifdef CONFIG_GPIO_KEYS_DIAG_TRIGGER
		.options		= OPT_DIAG_TRIGGER,
#endif
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
		.options		= 0,
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
		.options        = 0,
	},	
	[4] = {
		.code			= KEY_SLIDER_OPEN,
		.gpio			= GPIO_SLIDER_OPEN,
		.active_low		= 1,
		.desc			= "slider open",
		.debounce		= 10,
		.type			= EV_KEY,
		.wakeup			= 0,
#ifdef  CONFIG_GPIO_KEYS_DIAG_TRIGGER
		.options    	= OPT_DIAG_TRIGGER | OPT_DIAG_TRIGGER_EDGE,
#endif
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

static struct gpio_keys_platform_data shank_gpio_keys = {
	.buttons  = shank_gpio_keys_buttons,
	.nbuttons = ARRAY_SIZE(shank_gpio_keys_buttons),
};

static struct platform_device shank_gpio_keys_device = {
	.name = "gpio-keys",
	.id   = -1,
	.dev  = {
		.platform_data  = &shank_gpio_keys,
	},
};


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

	if(board_type < BW_DVT1) {
		return;
	}

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

#ifdef CONFIG_MSM_VIBRATOR
	pinmux_config("VIBRATOR_EN", PINMUX_CONFIG_SLEEP);
	pinmux_config("VIBRATOR_PWM", PINMUX_CONFIG_SLEEP);
#endif

	// I2C
	pinmux_config("I2C_SCL", PINMUX_CONFIG_SLEEP);
	pinmux_config("I2C_SDA", PINMUX_CONFIG_SLEEP);

	if (board_type < BW_EMU2) {
		pinmux_config("PROX_INT_EVT1", PINMUX_CONFIG_SLEEP);
		pinmux_config("MOTION_INT2_EVT1", PINMUX_CONFIG_SLEEP);
	} else {
		pinmux_config("PROX_INT", PINMUX_CONFIG_SLEEP);
		pinmux_config("MOTION_INT2", PINMUX_CONFIG_SLEEP);
	}

	pinmux_config("MOTION_INT1", PINMUX_CONFIG_SLEEP);

	pinmux_config("CTP_RESET", PINMUX_CONFIG_SLEEP);
	pinmux_config("CTP_SCLK", PINMUX_CONFIG_SLEEP);
	pinmux_config("CTP_MOSI", PINMUX_CONFIG_SLEEP);
	pinmux_config("CTP_MISO", PINMUX_CONFIG_SLEEP);
	pinmux_config("CTP_SHDN", PINMUX_CONFIG_SLEEP);

	if (board_type < BW_EMU2)
		pinmux_config("CTP_SS_EVT1", PINMUX_CONFIG_SLEEP);

	else
		pinmux_config("CTP_SS_EVT2", PINMUX_CONFIG_SLEEP);

	// A6 I2C
	pinmux_config("I2C_SCL_CHG", PINMUX_CONFIG_SLEEP);
	pinmux_config("I2C_SDA_CHG", PINMUX_CONFIG_SLEEP);

	pinmux_config_power_collapse("LDO1");
	pinmux_config_power_collapse("LDO2");
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

#ifdef CONFIG_MSM_VIBRATOR
	pinmux_config("VIBRATOR_EN", PINMUX_CONFIG_ACTIVE);
	pinmux_config("VIBRATOR_PWM", PINMUX_CONFIG_ACTIVE);
#endif

	// I2C
	pinmux_config("I2C_SCL", PINMUX_CONFIG_ACTIVE);
	pinmux_config("I2C_SDA", PINMUX_CONFIG_ACTIVE);

	if (board_type < BW_EMU2) {
		pinmux_config("PROX_INT_EVT1", PINMUX_CONFIG_ACTIVE);
		pinmux_config("MOTION_INT2_EVT1", PINMUX_CONFIG_ACTIVE);
	} else {
		pinmux_config("PROX_INT", PINMUX_CONFIG_ACTIVE);
		pinmux_config("MOTION_INT2", PINMUX_CONFIG_ACTIVE);
	}

	pinmux_config("MOTION_INT1", PINMUX_CONFIG_ACTIVE);

	pinmux_config("CTP_RESET", PINMUX_CONFIG_ACTIVE);
	pinmux_config("CTP_SCLK", PINMUX_CONFIG_ACTIVE);
	pinmux_config("CTP_MOSI", PINMUX_CONFIG_ACTIVE);
	pinmux_config("CTP_MISO", PINMUX_CONFIG_ACTIVE);
	pinmux_config("CTP_SHDN", PINMUX_CONFIG_ACTIVE);

	if (board_type < BW_EMU2)
		pinmux_config("CTP_SS_EVT1", PINMUX_CONFIG_ACTIVE);

	else
		pinmux_config("CTP_SS_EVT2", PINMUX_CONFIG_ACTIVE);
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
	
	pinmux_config("WIFI_INT", PINMUX_CONFIG_SLEEP);		// wifi sdio
	pinmux_config("WIFI_RESET", PINMUX_CONFIG_SLEEP);
	pinmux_config("WIFI_EN", PINMUX_CONFIG_SLEEP);
	pinmux_config("WL_SD_CLK", PINMUX_CONFIG_SLEEP);
	pinmux_config("WL_SD_CMD", PINMUX_CONFIG_SLEEP);
	pinmux_config("WL_SD_DATA3", PINMUX_CONFIG_SLEEP);
	pinmux_config("WL_SD_DATA2", PINMUX_CONFIG_SLEEP);
	pinmux_config("WL_SD_DATA1", PINMUX_CONFIG_SLEEP);
	pinmux_config("WL_SD_DATA0", PINMUX_CONFIG_SLEEP);

	pinmux_config("BT_RESET", PINMUX_CONFIG_ACTIVE);		// bluetooth 
	pinmux_config("BT_BT_WAKE_HOST", PINMUX_CONFIG_ACTIVE);

	// Bluetooth is on by default
	wlan_reset_pin_control(WLAN_BT, WLAN_RESET_ON);
	
#ifdef CONFIG_KEYBOARD_GPIO_PE
	pinmux_config("SLIDER_OPEN", PINMUX_CONFIG_ACTIVE);
	pinmux_config("SLIDER_CLOSE", PINMUX_CONFIG_ACTIVE);
	pinmux_config("RINGER", PINMUX_CONFIG_ACTIVE);
	pinmux_config("VOL_UP", PINMUX_CONFIG_ACTIVE);
	pinmux_config("VOL_DOWN", PINMUX_CONFIG_ACTIVE);
	pinmux_config("POWER", PINMUX_CONFIG_ACTIVE);
#endif

#ifdef CONFIG_MSM_VIBRATOR
	pinmux_config("VIBRATOR_EN", PINMUX_CONFIG_ACTIVE);
	pinmux_config("VIBRATOR_PWM", PINMUX_CONFIG_ACTIVE);
#endif

#ifdef CONFIG_LEDS_LP8501	
	pinmux_config("LP8501_EN", PINMUX_CONFIG_ACTIVE);
	pinmux_config("LP8501_INT", PINMUX_CONFIG_ACTIVE);
#endif

	pinmux_config("CTP_RESET", PINMUX_CONFIG_ACTIVE);
	pinmux_config("CTP_INT", PINMUX_CONFIG_ACTIVE);
	pinmux_config("CTP_SCLK", PINMUX_CONFIG_ACTIVE);
	pinmux_config("CTP_MOSI", PINMUX_CONFIG_ACTIVE);
	pinmux_config("CTP_MISO", PINMUX_CONFIG_ACTIVE);

	if (board_type < BW_EMU2)
		pinmux_config("CTP_SS_EVT1", PINMUX_CONFIG_ACTIVE);
	else
		pinmux_config("CTP_SS_EVT2", PINMUX_CONFIG_ACTIVE);

	// I2C
	pinmux_config("I2C_SCL", PINMUX_CONFIG_ACTIVE);
	pinmux_config("I2C_SDA", PINMUX_CONFIG_ACTIVE);

	if (board_type < BW_EMU2) {
		pinmux_config("PROX_INT_EVT1", PINMUX_CONFIG_ACTIVE);
		pinmux_config("MOTION_INT2_EVT1", PINMUX_CONFIG_ACTIVE);
	} else {
		pinmux_config("PROX_INT", PINMUX_CONFIG_ACTIVE);
		pinmux_config("MOTION_INT2", PINMUX_CONFIG_ACTIVE);
	}

	pinmux_config("MOTION_INT1", PINMUX_CONFIG_ACTIVE);
	pinmux_config("CTP_SHDN", PINMUX_CONFIG_ACTIVE);

	// A6 I2C
	pinmux_config("I2C_SCL_CHG", PINMUX_CONFIG_ACTIVE);
	pinmux_config("I2C_SDA_CHG", PINMUX_CONFIG_ACTIVE);

	pinmux_config("LDO1", PINMUX_CONFIG_ACTIVE);
	pinmux_config("LDO2", PINMUX_CONFIG_ACTIVE);
};


#endif

static struct pm8058_platform_data pm8058_7x30_data = {
	.pm_irqs = {
		[PM8058_IRQ_KEYPAD - PM8058_FIRST_IRQ] = 74,
		[PM8058_IRQ_KEYSTUCK - PM8058_FIRST_IRQ] = 75,
	},
	.init = pm8058_gpios_init,
	.num_subdevs = 2,
	.sub_devices = {
		{	
			.name	        = "pm8058-keypad",
			.num_resources	= ARRAY_SIZE(resources_keypad),
			.resources      = resources_keypad,
			.platform_data  = &broadway_keypad_data,
			.data_size      = sizeof(broadway_keypad_data)
			
		},
		{
			.name		= "pm8058-pwm",
		}
        }
};

static struct i2c_board_info pm8058_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("pm8058-core", 0),
		.irq = MSM_GPIO_TO_INT(PMIC_GPIO_INT),
		.platform_data = &pm8058_7x30_data,
	},
};

#ifdef CONFIG_MSM_CAMERA
static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
#ifdef CONFIG_VX6953
	{
		I2C_BOARD_INFO("vx6953", 	0x20),
	},
#endif
};

static uint32_t camera_off_gpio_table[] = {
	GPIO_CFG(15,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* MCLK */
	GPIO_CFG(109, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* cam_on */
};

static uint32_t camera_on_gpio_table[] = {
	// TODO:: some of these are not needed anymore for MIPI - 
	GPIO_CFG(15,  1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_4MA), 	/* MCLK */
	GPIO_CFG(109, 0, GPIO_OUTPUT, GPIO_PULL_UP,   GPIO_4MA), 	/* cam_on */
};


static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

#define SMPS_CAMERA_ID "CAMD"

static void config_camera_on_gpios(void)
{
	int rc;
	static int vregs_initialized = 0;
	struct vreg* vreg_wlan = NULL;
	struct vreg* vreg_gp2 = NULL;


	// keep S4 regulator in PWM mode
	rc = pmapp_smps_mode_vote(SMPS_CAMERA_ID, PMAPP_VREG_S4, PMAPP_SMPS_MODE_VOTE_PWM);
	if (rc != 0) {
		printk("pmapp_smps_mode_vote error %d\n", rc);
	}
	
	/* keep vregs rail on because of slow discharge problem */
	if (!vregs_initialized) {
		vreg_wlan = vreg_get(NULL, "wlan");
		vreg_gp2  = vreg_get(NULL, "gp2");
		if (IS_ERR(vreg_wlan) || IS_ERR(vreg_gp2)) {
			printk(KERN_ERR "%s: wlan get failed (%ld)\n",
					__func__, PTR_ERR(vreg_wlan));
			return ;
		}

		CDBG("config wlan\n");
		rc = vreg_set_level(vreg_wlan, 1800);
		if (rc) {
			printk(KERN_ERR "%s: lwan set level failed (%d)\n",
					__func__, rc);
			return ;
		}

		CDBG("config gp2\n");
		rc = vreg_set_level(vreg_gp2, 2850);
		if (rc) {
			printk(KERN_ERR "%s: gp2 set level failed (%d)\n",
					__func__, rc);
			return ;
		}

		CDBG("enable wlan\n");
		rc = vreg_enable(vreg_wlan);
		if (rc) {
			printk(KERN_ERR "%s: wlan enable failed (%d)\n",
					__func__, rc);
			return ;
		}

		CDBG("enable gp2\n");
		rc = vreg_enable(vreg_gp2);
		if (rc) {
			printk(KERN_ERR "%s: gp2 enable failed (%d)\n",
					__func__, rc);
			return ;
		}
		vregs_initialized = 1;
	}

	CDBG(KERN_ERR "CAMERA - enable gpios\n");
	gpio_set_value(109, 0);	
	config_gpio_table(camera_on_gpio_table,
				ARRAY_SIZE(camera_on_gpio_table));

	CDBG("set 109 - CAM_ON\n");
	gpio_set_value(109, 1);	
}

static void config_camera_off_gpios(void)
{
	int rc;
	CDBG("set 109 - CAM_OFF\n");
	gpio_set_value(109, 0);	

	CDBG(KERN_ERR "CAMERA - disable gpios\n");
	config_gpio_table(camera_off_gpio_table,
				ARRAY_SIZE(camera_off_gpio_table));

	// allow S4 to enter PFM mode when Camera is off.
	rc = pmapp_smps_mode_vote(SMPS_CAMERA_ID, PMAPP_VREG_S4, PMAPP_SMPS_MODE_VOTE_DONTCARE);
	if (rc != 0) {
		printk("pmapp_smps_mode_vote error %d\n", rc);
	}
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

struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
};


#if 0
static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PWM,
	._fsrc.pwm_src.freq  = 1000,
	._fsrc.pwm_src.max_load = 300,
	._fsrc.pwm_src.low_load = 30,
	._fsrc.pwm_src.high_load = 100,
	._fsrc.pwm_src.channel = 7,
};
#endif

#ifdef CONFIG_VX6953
static struct msm_camera_sensor_flash_data msm_flash_none = {
       .flash_type = MSM_CAMERA_FLASH_NONE,
       .flash_src  = NULL
};

static struct msm_camera_sensor_info msm_camera_sensor_vx6953_data = {
	.sensor_name    = "vx6953",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.pdata          = &msm_camera_device_data,
	.flash_data     = &msm_flash_none,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.csi_if             = 1,
    .mipi_data_swapped  = 0,
};

static struct platform_device msm_camera_sensor_vx6953 = {
	.name      = "msm_camera_vx6953",
	.dev       = {
		.platform_data = &msm_camera_sensor_vx6953_data,
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

#endif /*CONFIG_MSM_CAMERA*/

#if defined(CONFIG_MSM7KV2_AUDIO) \
	&& defined(CONFIG_MARIMBA_CODEC)
static uint32_t audio_pamp_gpio_config =
   GPIO_CFG(82, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA);

static int __init snddev_poweramp_gpio_init(void)
{
	int rc;

	pr_info("snddev_poweramp_gpio_init \n");
	rc = gpio_tlmm_config(audio_pamp_gpio_config, GPIO_ENABLE);
	if (rc) {
		printk(KERN_ERR
			"%s: gpio_tlmm_config(%#x)=%d\n",
			__func__, audio_pamp_gpio_config, rc);
	}
	return rc;
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

void msm_snddev_hsed_pamp_on(void)
{
	struct vreg *vreg_ncp;
	int rc;

	vreg_ncp = vreg_get(NULL, "ncp");
	if (IS_ERR(vreg_ncp)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(vreg_ncp));
		return;
	}
	rc = vreg_enable(vreg_ncp);
	if (rc)
		pr_err("%s: vreg_enable failed (%d)\n", __func__, rc);
}

void msm_snddev_hsed_pamp_off(void)
{
	struct vreg *vreg_ncp;
	int rc;

	vreg_ncp = vreg_get(NULL, "ncp");
	if (IS_ERR(vreg_ncp)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(vreg_ncp));
		return;
	}
	rc = vreg_disable(vreg_ncp);
	if (rc)
		pr_err("%s: vreg_disable failed (%d)\n", __func__, rc);
}
#endif // CONFIG_MSM7KV2_AUDIO && CONFIG_MARIMBA_CODEC

static struct msm_v4l2_pd msm_v4l2_device_data = {
   .screen_width = 320,
   .screen_height = 400,
   .max_internal_bufs = 3,
   .scaling_factor = 4,
};

static struct platform_device msm_v4l2_device = {
   .name = "msmv4l2_pd",
   .id   = -1,
   .dev  = {
      .platform_data = &msm_v4l2_device_data
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
	.init_timer = msm_hres_timer_init,
	.release_timer = msm_hres_timer_release,
	.suspend_timer = msm_hres_timer_suspend,
	.resume_timer = msm_hres_timer_resume,
	.read_timer = msm_hres_timer_read,
	.convert_timer = msm_hres_timer_convert,
};

static struct platform_device hres_counter_device = {
	.name = "hres_counter",
	.id   = -1,
	.dev  = {
		.platform_data  = &msm_hres_counter_platform_data,
	}
};
#endif

static unsigned aux_pcm_gpio_on[] = {
	GPIO_CFG(138, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),   /* PCM_DOUT */
	GPIO_CFG(139, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA),   /* PCM_DIN  */
	GPIO_CFG(140, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),   /* PCM_SYNC */
	GPIO_CFG(141, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),   /* PCM_CLK  */
};

static int __init aux_pcm_gpio_init(void)
{
	int pin, rc;

	pr_info("aux_pcm_gpio_init \n");
	for (pin = 0; pin < ARRAY_SIZE(aux_pcm_gpio_on); pin++) {
		rc = gpio_tlmm_config(aux_pcm_gpio_on[pin],
					GPIO_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, aux_pcm_gpio_on[pin], rc);
		}
	}
	return rc;
}

static int __init buses_init(void)
{
	if (gpio_tlmm_config(GPIO_CFG(PMIC_GPIO_INT, 1, GPIO_INPUT,
				  GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, PMIC_GPIO_INT);

	i2c_register_board_info(6 /* I2C_SSBI ID */, pm8058_boardinfo,
				ARRAY_SIZE(pm8058_boardinfo));

	return 0;
}

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
	0, 0, 0, 0,
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
/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
	{
		.product_id         = 0x9015,
		/* MSC + ADB */
		.functions	    = 0x12 /* 10010 */
	},
	{
		.product_id         = 0xF000,
		/* MSC */
		.functions	    = 0x02, /* 0010 */
	},
	{
		.product_id         = 0xF005,
		/* MODEM ONLY */
		.functions	    = 0x03,
	},

	{
		.product_id         = 0x8080,
		/* DIAG + MODEM */
		.functions	    = 0x34,
	},
	{
		.product_id         = 0x8082,
		/* DIAG + ADB + MODEM */
		.functions	    = 0x0314,
	},
	{
		.product_id         = 0x8085,
		/* DIAG + ADB + MODEM + NMEA + MSC*/
		.functions	    = 0x25314,
	},
	{
		.product_id         = 0x9016,
		/* DIAG + GENERIC MODEM + GENERIC NMEA*/
		.functions	    = 0x764,
	},
	{
		.product_id         = 0x9017,
		/* DIAG + GENERIC MODEM + GENERIC NMEA + MSC*/
		.functions	    = 0x2764,
	},
	{
		.product_id         = 0x9018,
		/* DIAG + ADB + GENERIC MODEM + GENERIC NMEA + MSC*/
		.functions	    = 0x27614,
	},
	{
		.product_id         = 0xF009,
		/* CDC-ECM*/
		.functions	    = 0x08,
	}
};
static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= 0x05C6,
	.product_id	= 0x9018,
	.functions	= 0x27614,
	.version	= 0x0100,
	.serial_number  = "1234567890ABCDEF",
	.compositions   = usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.product_name	= "Qualcomm HSUSB Device",
	.manufacturer_name = "Qualcomm Incorporated",
	.nluns = 1,
};
static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
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
};

static struct platform_device hs_device = {
	.name   = "headset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

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

#if 0
	spi_mux = (ioread32(ADMH0_GP_CTL) & (0x3 << 12)) >> 12;
#else
	// Palm hack: Force the mux to be 2 for now. SR has been opened with
	// Qualcomm to understand why the value in the original code is 0.
	spi_mux = 2;
	iowrite32(ioread32(ADMH0_GP_CTL) | (spi_mux << 12), ADMH0_GP_CTL);		
#endif

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

#ifdef CONFIG_TOUCHSCREEN_CY8MRLN
#include <linux/spi/cy8mrln.h>
static int cy8mrln_switch_mode(int flag)
{
	return (0);
}

static struct cy8mrln_platform_data msm_cy8mrln_data_evt1 =
{
	.enable_gpio = 128, // Same as XRES
	.external_cs = 132,
	.switch_mode = cy8mrln_switch_mode,
};

static struct cy8mrln_platform_data msm_cy8mrln_data_evt2 =
{
	.enable_gpio = 128, // Same as XRES
	.external_cs = 46,
	.switch_mode = cy8mrln_switch_mode,
};
#endif /* CONFIG_TOUCHSCREEN_CY8MRLN */

static struct spi_board_info msm_spi_board_info[] __initdata = {
#if defined(CONFIG_TOUCHSCREEN_CY8MRLN) \
	|| defined(CONFIG_TOUCHSCREEN_CY8MRLN_MODULE)
	{
		.modalias = "cy8mrln",
		.bus_num = 0,
		.chip_select = 2,
		.mode = SPI_CPHA,
		.max_speed_hz = 100000,
		.platform_data = NULL, // Set later on now
		.irq = MSM_GPIO_TO_INT(147),
	},
#elif defined(CONFIG_TOUCHSCREEN_CY8CTMA300) \
       || defined(CONFIG_TOUCHSCREEN_CY8CTMA300_MODULE)
	{
		.modalias = "spidev",
		.max_speed_hz = 1600000,
		.bus_num = 0,
		.chip_select = 2,
		.mode = SPI_MODE_0,
	},
#endif /* CONFIG_TOUCHSCREEN_CY8MRLN[_MODULE] */
};

#if defined(CONFIG_TOUCHSCREEN_CY8CTMA300) \
       || defined(CONFIG_TOUCHSCREEN_CY8CTMA300_MODULE)
static int shank_cy8ctma300_gpio_request(unsigned gpio, char *name)
{
	int rc;

	rc = gpio_request(gpio, name);
	if (rc < 0) {
		pr_err("error %d requesting gpio %u (%s)\n", rc, gpio, name);
		goto exit;
	}

	pinmux_config(name, PINMUX_CONFIG_ACTIVE);
exit:
	return (rc);
}

static int __init shank_cy8ctma300_init(void)
{
#if defined(CONFIG_TOUCHSCREEN_CY8MRLN) \
	|| defined(CONFIG_TOUCHSCREEN_CY8MRLN_MODULE)
	return (0);
#else /* !CONFIG_TOUCHSCREEN_CY8MRLN[_MODULE] */
	return (shank_cy8ctma300_gpio_request(GPIO_CTP_RESET, "CTP_RESET"));
#endif /* CONFIG_TOUCHSCREEN_CY8MRLN[_MODULE] */
}

/* this needs to be called after gpio init (postcore) but before device init */
arch_initcall(shank_cy8ctma300_init);

static int shank_cy8ctma300_sclk_request(int request)
{
	int rc;

	if (request)
		rc = shank_cy8ctma300_gpio_request(GPIO_CTP_SCLK,
							"CY8CTMA300_SCLK");

	else {
		pinmux_config("CTP_SCLK", PINMUX_CONFIG_ACTIVE);
		gpio_free(GPIO_CTP_SCLK);
		rc = 0;
	}

	return (rc);
}

static int shank_cy8ctma300_sdata_request(int request)
{
	int rc;

	if (request)
		rc = shank_cy8ctma300_gpio_request(GPIO_CTP_MOSI,
							"CY8CTMA300_SDATA");

	else {
		pinmux_config("CTP_MOSI", PINMUX_CONFIG_ACTIVE);
		gpio_free(GPIO_CTP_MOSI);
		rc = 0;
	}

	return (rc);
}

static void shank_cy8ctma300_xres_assert(int assert)
{
	(void)gpio_direction_output(GPIO_CTP_RESET, !assert);
}


static void shank_cy8ctma300_vcpin_enable(int enable)
{
 	(void)gpio_direction_output(GPIO_CTP_SHDN, enable);
}

static struct cy8ctma300_platform_data shank_cy8ctma300_data = {
	.prec_len = 64,
	.nr_srecs = 2,
	.block_len = 128,
	.nr_blocks = 256,
	.sclk_request = shank_cy8ctma300_sclk_request,
	.sdata_request = shank_cy8ctma300_sdata_request,
	.vdd_enable = NULL,
 	.vcpin_enable = shank_cy8ctma300_vcpin_enable,
	.xres_assert = shank_cy8ctma300_xres_assert,
	.sclk = GPIO_CTP_SCLK,
	.sdata = GPIO_CTP_MOSI,
	.xres_us = 263,
	.reset_ns = 500,
	.ssclk_ns = 40,
	.hsclk_ns = 40,
	.dsclk_ns = 70,
	.wait_and_poll_ms = 200,
};

static struct platform_device shank_cy8ctma300_device = {
	.name = CY8CTMA300_DEVICE,
	.id = -1,
	.dev = {
		.platform_data = &shank_cy8ctma300_data,
	},
};
#endif /* CONFIG_TOUCHSCREEN_CY8CTMA300[_MODULE] */

static struct msm_spi_platform_data qsd_spi_pdata = {
	.max_clock_speed = 3000000,
	.dma_config = msm_qsd_spi_dma_config,
};

static void __init msm_qsd_spi_init(void)
{
	qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}
#ifdef CONFIG_USB_ANDROID
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}

static int hsusb_chg_init(int connect)
{
	if (connect)
		return msm_chg_rpc_connect();
	else
		return msm_chg_rpc_close();
}

void hsusb_chg_vbus_draw(unsigned mA)
{
	if (mA)
		msm_chg_usb_i_is_available(mA);
	else
		msm_chg_usb_i_is_not_available();
}

void hsusb_chg_connected(enum chg_type chgtype)
{
	switch (chgtype) {
	case CHG_TYPE_HOSTPC:
		pr_debug("Charger Type: HOST PC\n");
		msm_chg_usb_charger_connected(0);
		msm_chg_usb_i_is_available(100);
		break;
	case CHG_TYPE_WALL_CHARGER:
		pr_debug("Charger Type: WALL CHARGER\n");
		msm_chg_usb_charger_connected(2);
		msm_chg_usb_i_is_available(1500);
		break;
	case CHG_TYPE_INVALID:
		pr_debug("Charger Type: DISCONNECTED\n");
		msm_chg_usb_i_is_not_available();
		msm_chg_usb_charger_disconnected();
		break;
	}
}

static int msm_hsusb_rpc_phy_reset(void __iomem *addr)
{
	return msm_hsusb_phy_reset();
}

static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect	= hsusb_rpc_connect,
	.phy_reset	= msm_hsusb_rpc_phy_reset,
	.core_clk	= 1,
};

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	/* charging apis */
	.chg_init = hsusb_chg_init,
	.chg_connected = hsusb_chg_connected,
	.chg_vbus_draw = hsusb_chg_vbus_draw,
};
#endif

#ifdef CONFIG_USB_GADGET
static struct msm_otg_platform_data msm_otg_pdata = {
	.core_clk	= 1,
};

static int shank_phy_init_seq[] = {
	/*(value, register) */
	0x2e, 0x31, /* PHY_CFG_REG_1 - amplitude 3 (I + 7.5%) */
	0x3f, 0x32, /* PHY_CFG_REG_2 - preemphasis depth=20% & slope control = 15 */
	-1
};

static void shank_phy_reset(void)
{
	/* put custom phy reset logic here if necessary */
}

static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.phy_init_seq = shank_phy_init_seq,
	.phy_reset = shank_phy_reset,
	.wall_charger_max_current_mA = 560,
};
#endif

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

#if defined(CONFIG_MSM_KGSL) && !defined(CONFIG_MSM_KGSL_MMU)
static struct android_pmem_platform_data android_pmem_gpu1_pdata = {
	.name = "pmem_gpu1",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_gpu1_device = {
	.name = "android_pmem",
	.id = 3,
	.dev = { .platform_data = &android_pmem_gpu1_pdata },
};
#endif

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
	.max_grp3d_freq = 245 * 1000*1000,
	.min_grp3d_freq = 192 * 1000*1000,
	.set_grp3d_async = set_grp3d_async,
};

#ifdef CONFIG_MSM_KGSL
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
#endif

#ifdef CONFIG_MSM_VIBRATOR

static struct vibrator_platform_data msm_vibe_platform_data= {
        .vibrator_enable_gpio = GPIO_VIBRATOR_EN,
        .power = NULL
};

static struct platform_device board_vibe_device = {
        .name = VIBE_DEVICE,
        .id   = -1,
        .dev  = {
                .platform_data  = &msm_vibe_platform_data,
        }
};

#endif

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = NULL,
	.mddi_sel_clk = NULL,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 24, //EVT1a
};

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", &mddi_pdata);
}

static struct leds_msm_pmic_data pmic_leds_data = {
	.max_current_mA = 10,
	.current_incr_mA = 2,
};

static struct platform_device msm_device_pmic_leds = {
	.name   = "pmic-leds",
	.id = -1,
	.dev  = {
		.platform_data  = &pmic_leds_data,
	},
};

static struct msm_ts_platform_data msm_ts_data = {
	.min_x          = 296,
	.max_x          = 3800,
	.min_y          = 296,
	.max_y          = 3800,
	.min_press      = 0,
	.max_press      = 256,
	.inv_x          = 4096,
	.inv_y          = 4096,
};


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
//	.rts_pin         = 145,   // uart rts line pin
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
 
static struct bluetooth_power_state_platform_data bluetooth_power_state_platform_data_broadway = {
        .dev_name       = "bt_power",
        .bt_power = bt_power,
};
  
static struct platform_device bluetooth_power_state_device = {
        .name   = "bt_power",
        .id     = 0,
        .dev    = { 
                .platform_data = &bluetooth_power_state_platform_data_broadway
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
		.name = "gpio_26",		// st331dlh - accelerometer int 1
		.gpio = GPIO_MOTION_INT1,
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
		.name = "gpio_130",		// st331dlh - accelerometer int 2
		.gpio = GPIO_MOTION_INT2,
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
		.gpio = GPIO_PROX_INT,
		.act_level = 1,
		.direction = 1,
		.def_level = 1,
		.pin_mode = (void*) -1,
		.sysfs_mask = 0777,
		.options = PIN_IRQ | PIN_WAKEUP_SOURCE,
		.irq_config = IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,	// both rising and falling 
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

#if !defined(CONFIG_TOUCHSCREEN_CY8MRLN) \
	&& !defined(CONFIG_TOUCHSCREEN_CY8MRLN_MODULE) \
	&& (defined(CONFIG_TOUCHSCREEN_CY8CTMA300) \
		|| defined(CONFIG_TOUCHSCREEN_CY8CTMA300_MODULE))

/*
 * Each touch generates roughly 5-6 interrupts, but we have no way of telling which is final, so we'll keep
 * setting a 1000ms limit on keeping the cpu frequency up. This ensures that when the gesture or tap is finished
 * we'll have the frequency high enough to render the gesture and then clock back down without lag
 */
irqreturn_t ctp_int_handler(int irq, void *data) {
	CPUFREQ_FLOOR_MILLIS(806400, 1000);
	return 0;
}

static struct user_pin ctp_pins[] = {
	{
		.name = "int",
		.gpio = GPIO_CTP_INT,
		.act_level = 1,
		.direction = 1,
		.def_level = -1,
		.sysfs_mask = 0440,
		.options = PIN_IRQ,
		.irq_config = IRQF_TRIGGER_RISING,
		.irq_handler = &ctp_int_handler,
	},
	{
		.name = "ss",
		.gpio = GPIO_CTP_SS_EVT2,
		.act_level = 1,
		.direction = 0,
		.def_level = 1,
		.sysfs_mask = 0220,
	},
};
#endif /* !CONFIG_TOUCHSCREEN_CY8MRLN[_MODULE]
		&& CONFIG_TOUCHSCREEN_CY8CTMA300[_MODULE] */

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
#if !defined(CONFIG_TOUCHSCREEN_CY8MRLN) \
	&& !defined(CONFIG_TOUCHSCREEN_CY8MRLN_MODULE) \
	&& (defined(CONFIG_TOUCHSCREEN_CY8CTMA300) \
		|| defined(CONFIG_TOUCHSCREEN_CY8CTMA300_MODULE))
	{
		.set_name = "ctp",
		.num_pins = ARRAY_SIZE(ctp_pins),
		.pins = ctp_pins,
	},
#endif /* !CONFIG_TOUCHSCREEN_CY8MRLN[_MODULE]
		&& CONFIG_TOUCHSCREEN_CY8CTMA300[_MODULE] */
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


static struct platform_device *devices[] __initdata = {
	&msm_device_smd,
	&msm_device_dmov,
#ifdef CONFIG_USB_FUNCTION
	&msm_device_hsusb_peripheral,
	&mass_storage_device,
#endif
#ifdef CONFIG_USB_GADGET
	&msm_device_otg,
	&msm_device_hsusb_peripheral,
	&msm_device_gadget_peripheral,
#endif
#ifdef CONFIG_USB_ANDROID
	&msm_device_otg,
	&msm_device_gadget_peripheral,
	&android_usb_device,
#endif
	&qsd_device_spi,
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi6,
	&msm_device_ssbi7,
#endif
	&msm_fb_device,
	&msm_rotator_device,
	&android_pmem_kernel_ebi1_device,
	&android_pmem_adsp_device,
	&msm_device_i2c,
	&msm_device_i2c_2,

#ifdef CONFIG_MSM_UARTDM
	&msm_device_uart_dm1,
#endif // Of CONFIG_MSM_UARTDM
#ifdef CONFIG_HSUART
	&btuart_device,
#endif // Of HSUART
	&bluetooth_power_state_device,

#ifdef CONFIG_USER_PINS
	&board_user_pins_device,
#endif

	&hs_device,
	&msm_aictl_device,
	&msm_mi2s_device,
	&msm_lpa_device,
	&msm_device_adspdec,
	&qup_device_i2c,
    
#if defined(CONFIG_MSM_KGSL)
#if !defined(CONFIG_MSM_KGSL_MMU)        
	&android_pmem_gpu1_device,
#endif
	&msm_device_kgsl,
#endif
    
#if defined(CONFIG_SERIAL_MSM) || defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart3,
#endif
	&msm_device_pmic_leds,
	&msm_device_tssc,
	&msm_device_vidc_720p,
	&msm_aux_pcm_device,
#ifdef CONFIG_KEYBOARD_GPIO_PE
	&shank_gpio_keys_device,
#endif
#ifdef CONFIG_HRES_COUNTER
	&hres_counter_device,
#endif
    &msm_v4l2_device,
#if defined(CONFIG_TOUCHSCREEN_CY8CTMA300) || defined(CONFIG_TOUCHSCREEN_CY8CTMA300_MODULE)
    &shank_cy8ctma300_device,
#endif

#ifdef CONFIG_NDUID
	&nduid_device,
#endif
#ifdef CONFIG_MSM_VIBRATOR
        &board_vibe_device,
#endif
#ifdef CONFIG_VX6953
	&msm_camera_sensor_vx6953,
#endif
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
};

static struct msm_gpio msm_i2c_gpios_hw[] = {
	{ GPIO_CFG(70, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA), "i2c_scl" },
	{ GPIO_CFG(71, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA), "i2c_sda" },
};

static struct msm_gpio msm_i2c_gpios_io[] = {
	{ GPIO_CFG(70, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), "i2c_scl" },
	{ GPIO_CFG(71, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), "i2c_sda" },
};

static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(16, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_10MA), "qup_scl" },
	{ GPIO_CFG(17, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_10MA), "qup_sda" },
};
static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(16, 2, GPIO_INPUT, GPIO_NO_PULL, GPIO_10MA), "qup_scl" },
	{ GPIO_CFG(17, 2, GPIO_INPUT, GPIO_NO_PULL, GPIO_10MA), "qup_sda" },
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
	.clk_freq = 400000,
	.rmutex  = 0,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
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
static struct msm_i2c_platform_data msm_i2c_ssbi6_pdata = {
	.rsl_id = "D:PMIC_SSBI"
};

static struct msm_i2c_platform_data msm_i2c_ssbi7_pdata = {
	.rsl_id = "D:CODEC_SSBI"
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

static void shank_probe_wifi(int id, struct mmc_host *mmc)
{
	printk("%s: id %d mmc %p\n", __PRETTY_FUNCTION__, id, mmc);
	wifi_mmc = mmc;
}

static void shank_remove_wifi(int id, struct mmc_host *mmc)
{
	printk("%s: id %d mmc %p\n", __PRETTY_FUNCTION__, id, mmc);
	wifi_mmc = NULL;
}

/*
 *  An API to enable wifi
 */
int board_sdio_wifi_enable(unsigned int param)
{
	printk(KERN_ERR "board_sdio_wifi_enable\n");
	wlan_reset_pin_control(WLAN_WIFI, WLAN_RESET_ON);
	pinmux_config("WIFI_EN", PINMUX_CONFIG_ACTIVE);
	pinmux_config("WIFI_INT", PINMUX_CONFIG_ACTIVE);

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
	printk(KERN_ERR "board_sdio_wifi_disable\n");

	//Put WiFi chip in RESET in order for the detection to fail and remove the card
	gpio_set_value(GPIO_WIFI_EN, 0);

	wlan_reset_pin_control(WLAN_WIFI, WLAN_RESET_OFF);
	pinmux_config("WIFI_EN", PINMUX_CONFIG_SLEEP);
	pinmux_config("WIFI_INT", PINMUX_CONFIG_SLEEP);
	if (wifi_mmc) {
		mmc_detect_change(wifi_mmc, msecs_to_jiffies(100));
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

	//printk(KERN_INFO "%s: vdd %d; vreg_sts:%lu id: %d\n", __func__, vdd, vreg_sts, pdev->id);

	if (vdd == 0) {

		msm_sdcc_setup_gpio(dev_id, !!vdd);

		if (!vreg_sts)
			return 0;
		
		if (dev_id == MMC_DEV_ID_MMC && MMC_DEV_IS_VREG_ON(dev_id)) {
			if (board_type > BW_EVT1) {
				pinmux_config("MMC_PWR_EVT2", PINMUX_CONFIG_SLEEP);
			}
		}


		clear_bit(dev_id, &vreg_sts);

		return 0;
	}
	else
	{
		if (dev_id == MMC_DEV_ID_MMC && MMC_DEV_IS_VREG_OFF(dev_id)) {
			if (board_type > BW_EVT1) {
				pinmux_config("MMC_PWR_EVT2", PINMUX_CONFIG_ACTIVE);
				gpio_set_value(GPIO_MMC_PWR_EVT2, 0);
			}
		}

		set_bit(dev_id, &vreg_sts);

		msm_sdcc_setup_gpio(dev_id, !!vdd);
	}


	return 0;
}

static struct mmc_platform_data msm7x30_sdc2_mmc_data = {
	.ocr_mask	= MMC_VDD_165_195,
	.translate_vdd	= msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
};

static struct mmc_platform_data msm7x30_sdc3_wifi_data = {
	.ocr_mask	= MMC_VDD_20_21,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.board_probe = shank_probe_wifi,
	.board_remove = shank_remove_wifi,
	.dummy52_required = 1,
};

static void __init msm7x30_init_mmc(void)
{
	// MMC
	msm_add_sdcc(2, &msm7x30_sdc2_mmc_data);

	// Wifi
	msm_add_sdcc(3, &msm7x30_sdc3_wifi_data);

}

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

#ifdef CONFIG_LEDS_LP8501 

/* Divides LED into group:
 * group 1: LCD
 * group 2: Core-Navi-Left
 * group 3: Core-Navi-Center
 * group 4: Core-Navi-Right
 */

static struct led_cfg lcd_group[] = {
	[0] = {
		.type = WHITE,
		.pwm_addr = D1_PWM,
		.current_addr = D1_CURRENT_CTRL,
		.control_addr = D1_CONTROL,
	},
	[1] = {
		.type = WHITE,
		.pwm_addr = D2_PWM,
		.current_addr = D2_CURRENT_CTRL,
		.control_addr = D2_CONTROL,
	},
	[2] = {
		.type = WHITE,
		.pwm_addr = D3_PWM,
		.current_addr = D3_CURRENT_CTRL,
		.control_addr = D3_CONTROL,
	},
	[3] = {
		.type = WHITE,
		.pwm_addr = D4_PWM,
		.current_addr = D4_CURRENT_CTRL,
		.control_addr = D4_CONTROL,
	},

	[4] = {
		.type = WHITE,
		.pwm_addr = D5_PWM,
		.current_addr = D5_CURRENT_CTRL,
		.control_addr = D5_CONTROL,
	},
};



static struct led_cfg core_navi_left_group[] = {
	[0] = {
		.type  = WHITE,
		.pwm_addr = D7_PWM,
		.current_addr = D8_CURRENT_CTRL,
		.control_addr = D8_CONTROL,
	},
};

static struct led_cfg core_navi_center_group[] = {
	[0] = {
		.type  = WHITE,
		.pwm_addr = D8_PWM,
		.current_addr = D7_CURRENT_CTRL,
		.control_addr = D7_CONTROL,
	},
};

static struct led_cfg core_navi_right_group[] = {
	[0] = {
		.type  = WHITE,
		.pwm_addr = D9_PWM,
		.current_addr = D9_CURRENT_CTRL,
		.control_addr = D9_CONTROL,
	},
};

static struct lp8501_led_config led_lp8501_data[] = {
	[GRP_1] = {
		.cdev = {
			.name = "lcd",
		},
		.led_list = &lcd_group[0],
		.nleds = ARRAY_SIZE(lcd_group),
		.group_id = GRP_1,
		.hw_group = HW_GRP_1,
		.default_current = 120, // 12.0mA
		.default_brightness = 40, // At 40%, each LED draws 4.8mA
		.default_state = LED_ON,
	},
	
	[GRP_2] = {
		.cdev = {
			.name = "core_navi_left",
		},
		.led_list = &core_navi_left_group[0],
		.nleds = ARRAY_SIZE(core_navi_left_group),
		.group_id = GRP_3,
		.hw_group = HW_GRP_NONE,
		.default_current  = 0x32, //5mA
		.default_brightness = 0,
		.default_state = LED_OFF,
	},
	[GRP_3] = {
		.cdev = {
			.name = "core_navi_center",
		},
		.led_list = &core_navi_center_group[0],
		.nleds = ARRAY_SIZE(core_navi_center_group),
		.group_id = GRP_4,
		.hw_group = HW_GRP_NONE,
		.default_current  = 0x32, //5mA
		.default_brightness = 0,
		.default_state = LED_OFF,
	},
	[GRP_4] = {
		.cdev = {
			.name = "core_navi_right",
		},
		.led_list = &core_navi_right_group[0],
		.nleds = ARRAY_SIZE(core_navi_right_group),
		.group_id = GRP_5,
		.hw_group = HW_GRP_NONE,
		.default_current  = 0x32, //5mA
		.default_brightness = 0,
		.default_state = LED_OFF,
	},
};

static struct lp8501_memory_config led_lp8501_memcfg = {
	.eng1_startpage = 0,
	.eng1_endpage = 1,
	.eng2_startpage = 2,
	.eng2_endpage = 3,
	.eng3_startpage = 4,
	.eng3_endpage = 5,
};

static struct lp8501_platform_data board_lp8501_data = {
	.leds = led_lp8501_data,
	.memcfg = &led_lp8501_memcfg,
	.nleds = ARRAY_SIZE(led_lp8501_data),
	.cp_mode = CONFIG_CPMODE_1x,
	.power_mode = CONFIG_POWER_SAVE_ON,
	.dev_name = "national_led",
};

static struct i2c_board_info lp8501_i2c_board_info_evt1 = {
	I2C_BOARD_INFO(LP8501_I2C_DEVICE, LP8501_I2C_ADDR),
	.platform_data = &board_lp8501_data,
	.irq = MSM_GPIO_TO_INT(GPIO_LP8501_INT),
};

#endif // CONFIG_LEDS_LP8501

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

	{GPIO_CFG(58, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "A6_TCK"},
	{GPIO_CFG(51, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), "A6_WAKEUP"},
	/* the pull-up config below is for input mode of A6_TDIO. A6 does not always drive
	   this and it floats in input mode unless explicitly pulled-up by host. */
	{GPIO_CFG(59, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA), "A6_TDIO"},
};

static struct msm_gpio a6_sbw_deinit_gpio_config_evt1[] = {

	{GPIO_CFG(58, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "A6_TCK"},
	{GPIO_CFG(51, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "A6_WAKEUP"},
	{GPIO_CFG(59, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "A6_TDIO"},
};

static struct msm_gpio a6_sbw_init_gpio_config_evt2[] = {
	{GPIO_CFG(58, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "A6_TCK"},
	{GPIO_CFG(165, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), "A6_WAKEUP"},
	/* the pull-up config below is for input mode of A6_TDIO. A6 does not always drive
	   this and it floats in input mode unless explicitly pulled-up by host. */
	{GPIO_CFG(59, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA), "A6_TDIO"},
};

static struct msm_gpio a6_sbw_deinit_gpio_config_evt2[] = {
	{GPIO_CFG(58, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "A6_TCK"},
	{GPIO_CFG(165, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "A6_WAKEUP"},
	{GPIO_CFG(59, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "A6_TDIO"},
};

static struct msm_gpio a6_sbw_init_gpio_config_dvt1[] = {
	{GPIO_CFG(61, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), "A6_TCK"},
	{GPIO_CFG(165, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), "A6_WAKEUP"},
	/* the pull-up config below is for input mode of A6_TDIO. A6 does not always drive
	   this and it floats in input mode unless explicitly pulled-up by host. */
	{GPIO_CFG(62, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA), "A6_TDIO"},
};

static struct msm_gpio a6_sbw_deinit_gpio_config_dvt1[] = {
	{GPIO_CFG(61, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "A6_TCK"},
	{GPIO_CFG(165, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "A6_WAKEUP"},
	{GPIO_CFG(62, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), "A6_TDIO"},
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

static struct a6_platform_data shank_a6_platform_data_evt1_0 = {
	.dev_name		= A6_DEVICE,
	.pwr_gpio       	= 165,
	.sbw_tck_gpio		= 58,
	.sbw_tdio_gpio		= 59,
	.sbw_wkup_gpio		= 51,
	.sbw_ops		= &sbw_ops_impl_0,

	.sbw_init_gpio_config	= a6_sbw_init_gpio_config_evt1,
	.sbw_init_gpio_config_size = ARRAY_SIZE(a6_sbw_init_gpio_config_evt1),
	.sbw_deinit_gpio_config	= a6_sbw_deinit_gpio_config_evt1,
	.sbw_deinit_gpio_config_size = ARRAY_SIZE(a6_sbw_deinit_gpio_config_evt1),

	.sbw_init		= a6_sbw_init_imp,
	.sbw_deinit		= a6_sbw_deinit_imp,
};

static struct a6_platform_data shank_a6_platform_data_evt2_0 = {
	.dev_name		= A6_DEVICE,
	.pwr_gpio       	= 51,
	.sbw_tck_gpio		= 58,
	.sbw_tdio_gpio		= 59,
	.sbw_wkup_gpio		= 165,
	.sbw_ops		= &sbw_ops_impl_0,

	.sbw_init_gpio_config	= a6_sbw_init_gpio_config_evt2,
	.sbw_init_gpio_config_size = ARRAY_SIZE(a6_sbw_init_gpio_config_evt2),
	.sbw_deinit_gpio_config	= a6_sbw_deinit_gpio_config_evt2,
	.sbw_deinit_gpio_config_size = ARRAY_SIZE(a6_sbw_deinit_gpio_config_evt2),

	.sbw_init		= a6_sbw_init_imp,
	.sbw_deinit		= a6_sbw_deinit_imp,
};

static struct a6_platform_data shank_a6_platform_data_dvt1_0 = {
	.dev_name		= A6_DEVICE,
	.pwr_gpio       	= 51,
	.sbw_tck_gpio		= 61,
	.sbw_tdio_gpio		= 62,
	.sbw_wkup_gpio		= 165,
	.sbw_ops		= &sbw_ops_impl_0,

	.sbw_init_gpio_config	= a6_sbw_init_gpio_config_dvt1,
	.sbw_init_gpio_config_size = ARRAY_SIZE(a6_sbw_init_gpio_config_dvt1),
	.sbw_deinit_gpio_config	= a6_sbw_deinit_gpio_config_dvt1,
	.sbw_deinit_gpio_config_size = ARRAY_SIZE(a6_sbw_deinit_gpio_config_dvt1),

	.sbw_init		= a6_sbw_init_imp,
	.sbw_deinit		= a6_sbw_deinit_imp,
};


static struct msm_gpio a6_config_data_evt1[] = {
	{GPIO_CFG(165, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), "A6_MSM_IRQ"}
};

static struct msm_gpio a6_config_data_evt2[] = {
	{GPIO_CFG(51, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), "A6_MSM_IRQ"}
};


static struct i2c_board_info a6_i2c_board_info_0 = {
	I2C_BOARD_INFO( A6_DEVICE, (0x62>>1)),
	.platform_data = NULL,
};

#if defined A6_PMIC_EXTERNAL_WAKE
static struct a6_pmic_wake_interface_data {
	int	pwm_channel;
	int	pwm_period;
	int	pwm_duty;
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
	int	wake_enable: 1;
	int	wake_period: 9;
	int	wake_gpio;
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

static void __init shank_init_a6(void)
{
	struct msm_gpio* config_data;
	int32_t config_data_size;
	
	printk(KERN_ERR "Registering a6_0 device.\n");
	if (board_type < BW_EMU2) {
		a6_i2c_board_info_0.platform_data = &shank_a6_platform_data_evt1_0;
	}
	else if (board_type < BW_DVT1){
		a6_i2c_board_info_0.platform_data = &shank_a6_platform_data_evt2_0;
	}
	else {
		a6_i2c_board_info_0.platform_data = &shank_a6_platform_data_dvt1_0;
	}
	i2c_register_board_info(0, &a6_i2c_board_info_0, 1);

	config_data = (board_type < BW_EMU2) ?
			a6_sbw_init_gpio_config_evt1 :
			((board_type < BW_DVT1) ?
				a6_sbw_init_gpio_config_evt2 :
				a6_sbw_init_gpio_config_dvt1);
	config_data_size = (board_type < BW_EMU2) ?
			ARRAY_SIZE(a6_sbw_init_gpio_config_evt1) :
			((board_type < BW_DVT1) ?
				ARRAY_SIZE(a6_sbw_init_gpio_config_evt2) :
				ARRAY_SIZE(a6_sbw_init_gpio_config_dvt1));
	msm_gpios_enable(config_data, config_data_size);

	/* no change for dvt boards */
	config_data = (board_type < BW_EMU2) ? a6_config_data_evt1 : a6_config_data_evt2;
	config_data_size = (board_type < BW_EMU2) ?
			ARRAY_SIZE(a6_config_data_evt1) : ARRAY_SIZE(a6_config_data_evt2);
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

	sbw_ops_impl_0.a6_per_device_interface.SetSBWTCK =
			(board_type < BW_DVT1) ? &a6_set_sbwtck_0 : &a6_set_sbwtck_dvt1_0;
	sbw_ops_impl_0.a6_per_device_interface.ClrSBWTCK =
			(board_type < BW_DVT1) ? &a6_clr_sbwtck_0 : &a6_clr_sbwtck_dvt1_0;
	sbw_ops_impl_0.a6_per_device_interface.SetSBWTDIO =
			(board_type < BW_DVT1) ? &a6_set_sbwtdio_0 : &a6_set_sbwtdio_dvt1_0;
	sbw_ops_impl_0.a6_per_device_interface.ClrSBWTDIO =
			(board_type < BW_DVT1) ? &a6_clr_sbwtdio_0 : &a6_clr_sbwtdio_dvt1_0;
	sbw_ops_impl_0.a6_per_device_interface.SetInSBWTDIO =
			(board_type < BW_DVT1) ? &a6_set_in_sbwtdio_0 : &a6_set_in_sbwtdio_dvt1_0;
	sbw_ops_impl_0.a6_per_device_interface.SetOutSBWTDIO =
			(board_type < BW_DVT1) ? &a6_set_out_sbwtdio_0 : &a6_set_out_sbwtdio_dvt1_0;
	sbw_ops_impl_0.a6_per_device_interface.GetSBWTDIO =
			(board_type < BW_DVT1) ? &a6_get_sbwtdio_0 : &a6_get_sbwtdio_dvt1_0;
	sbw_ops_impl_0.a6_per_device_interface.SetSBWAKEUP =
			(board_type < BW_EMU2) ? &a6_set_sbwakeup_evt1_0 : &a6_set_sbwakeup_evt2_0;
	sbw_ops_impl_0.a6_per_device_interface.ClrSBWAKEUP =
			(board_type < BW_EMU2) ? &a6_clr_sbwakeup_evt1_0 : &a6_clr_sbwakeup_evt2_0;
	sbw_ops_impl_0.a6_per_target_interface.delay = a6_delay_impl;
}
#endif // CONFIG_A6

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

static void __init msm7x30_init(void)
{
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);
	msm_clock_init(msm_clocks_7x30, msm_num_clocks_7x30);
	msm_spm_init(&msm_spm_data, 1);
	msm_acpu_clock_init(&msm7x30_clock_data);

	board_shank_gpios_init();

	msm_board_mux_init();

#ifdef CONFIG_VX6953
	// in EVT1b and EVT1a MIPI_DATA_N and MIPI_DATA_P are swapped.
	// EMU2 has the same connectivity as 1b despite being EVT2 based
	if (board_type < BW_EVT2 || board_type == BW_EMU2) {
		msm_camera_sensor_vx6953_data.mipi_data_swapped = 1;
	}
	else {
		msm_camera_sensor_vx6953_data.mipi_data_swapped = 0;
	}

	CDBG("CAMERA - swap mipi data line %d\n", msm_camera_sensor_vx6953_data.mipi_data_swapped);
#endif 

#ifdef CONFIG_USB_FUNCTION
	msm_hsusb_pdata.swfi_latency =
		msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;
#endif
#if defined(CONFIG_USB_ANDROID) || defined(CONFIG_USB_GADGET)
	msm_gadget_pdata.swfi_latency =
		msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#endif
#if !defined(CONFIG_TOUCHSCREEN_CY8MRLN) \
	&& !defined(CONFIG_TOUCHSCREEN_CY8MRLN_MODULE) \
	&& (defined(CONFIG_TOUCHSCREEN_CY8CTMA300) \
		|| defined(CONFIG_TOUCHSCREEN_CY8CTMA300_MODULE))
	if (board_type < BW_EMU2) {
		int i;

		for (i = 0; i < ARRAY_SIZE(ctp_pins); i++) {
			if (!strcmp(ctp_pins[i].name, "ss")) {
				ctp_pins[i].gpio = GPIO_CTP_SS_EVT1;
				break;
			}
		}
	}
#endif /* !CONFIG_TOUCHSCREEN_CY8MRLN[_MODULE]
		&& CONFIG_TOUCHSCREEN_CY8CTMA300[_MODULE] */
#ifdef CONFIG_USER_PINS
	if (board_type < BW_EMU2) {
		int i;
		for (i = 0; i < ARRAY_SIZE(usd_pins); i++) {
			if (usd_pins[i].gpio == GPIO_MOTION_INT2) {
				usd_pins[i].name = "gpio_18";
				usd_pins[i].gpio = GPIO_MOTION_INT2_EVT1;
				continue;
			}
			if (usd_pins[i].gpio == GPIO_PROX_INT) {
				usd_pins[i].name = "gpio_130";
				usd_pins[i].gpio = GPIO_PROX_INT_EVT1;
				continue;
			}
		}
	}
#endif // #ifdef CONFIG_USER_PINS
	platform_add_devices(devices, ARRAY_SIZE(devices));
	rmt_storage_add_ramfs();
	msm7x30_init_mmc();
	msm_qsd_spi_init();
#ifdef CONFIG_TOUCHSCREEN_CY8MRLN	
	if (board_type < BW_EMU2)
		msm_spi_board_info->platform_data = &msm_cy8mrln_data_evt1;
	else
		msm_spi_board_info->platform_data = &msm_cy8mrln_data_evt2;

	spi_register_board_info(msm_spi_board_info,
		ARRAY_SIZE(msm_spi_board_info));
#elif defined(CONFIG_TOUCHSCREEN_CY8CTMA300) \
       || defined(CONFIG_TOUCHSCREEN_CY8CTMA300_MODULE)
	spi_register_board_info(msm_spi_board_info,
				ARRAY_SIZE(msm_spi_board_info));
#endif /* CONFIG_TOUCHSCREEN_CY8MRLN */
	msm_fb_add_devices();
	msm_pm_set_platform_data(msm_pm_data);
	msm_device_i2c_init();
	msm_device_i2c_2_init();
	qup_device_i2c_init();
	buses_init();
#ifdef CONFIG_MSM7KV2_AUDIO
#ifdef CONFIG_MARIMBA_CODEC
	snddev_poweramp_gpio_init();
#endif
	msm_snddev_init();
#endif

#ifdef CONFIG_LEDS_LP8501	
	i2c_register_board_info(4, &lp8501_i2c_board_info_evt1, 1);
#endif

#ifdef CONFIG_MSM_CAMERA
	CDBG("i2c_register_board_info(2)\n");
	i2c_register_board_info(	2, 
					msm_camera_boardinfo,
					ARRAY_SIZE(msm_camera_boardinfo));
	CDBG("i2c_register_board_info(2) - done\n");
#endif


#if 0
#if defined(CONFIG_SERIAL_MSM) || defined(CONFIG_MSM_SERIAL_DEBUGGER)
	msm7x30_init_uart2();
#endif
#endif
	msm_device_tssc.dev.platform_data = &msm_ts_data;
#ifdef CONFIG_I2C_SSBI
	msm_device_ssbi6.dev.platform_data = &msm_i2c_ssbi6_pdata;
	msm_device_ssbi7.dev.platform_data = &msm_i2c_ssbi7_pdata;
#endif
	aux_pcm_gpio_init();

	if (board_type <= BW_EVT2) {
		printk("msm_pmic_leds: Setting current limit to 10mA "
		       "and current stepping to 10mA\n");
		pmic_leds_data.max_current_mA = 10;
		pmic_leds_data.current_incr_mA = 10;
	}

#ifdef CONFIG_A6
	shank_init_a6();
#endif // CONFIG_A6


}

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

	/* reserve the first page for suspend/resume */
	reserve_bootmem(0, 4096, BOOTMEM_EXCLUSIVE);

	size = MSM_FB0_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb0\n",
		size, addr, __pa(addr));

	size = MSM_FB1_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[1].start = __pa(addr);
	msm_fb_resources[1].end = msm_fb_resources[1].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb1\n",
		size, addr, __pa(addr));

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
    
	size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}

}
/* Board type selection */
static int  __init board_args(char *str)
{
    if (!strcmp(str, "broadway-proto")) {
        board_type = BW_PROTO;
    } else if (!strcmp(str, "broadway-evt1")) {
        board_type = BW_EVT1;
    } else if (!strcmp(str, "broadway-evt2")) {
        board_type = BW_EVT2;
    } else if (!strcmp(str, "broadway-emu2")) {
        board_type = BW_EMU2;
    } else if (!strcmp(str, "broadway-evt3")) {
        board_type = BW_EVT3;
    } else if (!strcmp(str, "broadway-dvt")) { // legacy
        board_type = BW_DVT1;
    } else if (!strcmp(str, "broadway-dvt1")) {
        board_type = BW_DVT1;
    } else if (!strcmp(str, "broadway-dvt2")) {
        board_type = BW_DVT2;
    } else if (!strcmp(str, "broadway-dvt3")) {
        board_type = BW_DVT3;
    } else if (!strcmp(str, "broadway-pvt")) {
        board_type = BW_PVT;
    } else {
        board_type = BW_PROTO;
    }    
	pr_info("system hardware revision: %d\n", board_type);
    return 0;
}

__setup("boardtype=", board_args);

static void __init msm7x30_map_io(void)
{
	msm_shared_ram_phys = 0x0FF00000;
	msm_map_msm7x30_io();
	msm7x30_allocate_memory_regions();
}

static void	__init shank_fixup(struct machine_desc *m, struct tag *t,
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

MACHINE_START(SHANK, "Shank")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = 0x00200100,
	.fixup = shank_fixup,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END

