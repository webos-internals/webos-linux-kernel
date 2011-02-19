#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <mach/gpio.h>




uint16_t a6_set_sbwtck_0(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(58, 1);
	return 0;
}

uint16_t a6_set_sbwtck_dvt1_0(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(61, 1);
	return 0;
}

uint16_t a6_clr_sbwtck_0(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(58, 0);
	return 0;
}

uint16_t a6_clr_sbwtck_dvt1_0(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(61, 0);
	return 0;
}

uint16_t a6_set_sbwtdio_0(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(59, 1);
	return 0;
}

uint16_t a6_set_sbwtdio_dvt1_0(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(62, 1);
	return 0;
}

uint16_t a6_clr_sbwtdio_0(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(59, 0);
	return 0;
}

uint16_t a6_clr_sbwtdio_dvt1_0(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(62, 0);
	return 0;
}

uint16_t a6_set_in_sbwtdio_0(void)
{
	gpio_direction_input(59);
	return 0;
}

uint16_t a6_set_in_sbwtdio_dvt1_0(void)
{
	gpio_direction_input(62);
	return 0;
}

uint16_t a6_set_out_sbwtdio_0(void)
{
	gpio_direction_output(59, 0);
	return 0;
}

uint16_t a6_set_out_sbwtdio_dvt1_0(void)
{
	gpio_direction_output(62, 0);
	return 0;
}

uint16_t a6_get_sbwtdio_0(void)
{
	return gpio_get_value(59);
}

uint16_t a6_get_sbwtdio_dvt1_0(void)
{
	return gpio_get_value(62);
}

uint16_t a6_set_sbwakeup_evt1_0(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(51, 1);
	return 0;
}

uint16_t a6_clr_sbwakeup_evt1_0(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(51, 0);
	return 0;
}

uint16_t a6_set_sbwakeup_evt2_0(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(165, 1);
	return 0;
}

uint16_t a6_clr_sbwakeup_evt2_0(void)
{
	// gpio_set_value configures the pin as gpio-output before writing value
	gpio_set_value(165, 0);
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
