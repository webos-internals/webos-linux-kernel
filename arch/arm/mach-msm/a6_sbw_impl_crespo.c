#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <mach/gpio.h>

#include "gpiomux-crespo.h"

/*
CRESPO_A6_0_TCK			156
CRESPO_A6_0_WAKEUP		155
CRESPO_A6_0_TDIO		120
CRESPO_A6_0_MSM_IRQ		37
*/

uint16_t a6_0_set_sbwtck(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(CRESPO_A6_0_TCK, 1);
	return 0;
}

uint16_t a6_0_clr_sbwtck(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(CRESPO_A6_0_TCK, 0);
	return 0;
}

uint16_t a6_0_set_sbwtdio(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(CRESPO_A6_0_TDIO, 1);
	return 0;
}

uint16_t a6_0_clr_sbwtdio(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(CRESPO_A6_0_TDIO, 0);
	return 0;
}

uint16_t a6_0_set_in_sbwtdio(void)
{
	gpio_direction_input(CRESPO_A6_0_TDIO);
	return 0;
}

uint16_t a6_0_set_out_sbwtdio(void)
{
	gpio_direction_output(CRESPO_A6_0_TDIO, 0);
	return 0;
}

uint16_t a6_0_get_sbwtdio(void)
{
	return gpio_get_value(CRESPO_A6_0_TDIO);
}

uint16_t a6_0_set_sbwakeup(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(CRESPO_A6_0_WAKEUP, 1);
	return 0;
}

uint16_t a6_0_clr_sbwakeup(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(CRESPO_A6_0_WAKEUP, 0);
	return 0;
}

/****/
/* per-target functions */
/****/
// delay in usecs
void a6_delay_impl(uint32_t delay_us)
{
	if ((delay_us >> 10) <= MAX_UDELAY_MS) {
		udelay(delay_us);
	}
	else {
		mdelay(delay_us >> 10);
	}
}

