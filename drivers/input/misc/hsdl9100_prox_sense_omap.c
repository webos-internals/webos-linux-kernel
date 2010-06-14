/*
 * linux/drivers/misc/omap_prox_sense_plat.c
 *
 * Platform dependent file for the HSDL9100 Proximity Sensor.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <asm/arch/dmtimer.h>
#include <asm/arch/gpio.h>
#include <linux/hsdl9100_proximity_sensor.h>
#include <linux/delay.h>

#undef PROX_SENSE_DEBUG
//#define PROX_SENSE_DEBUG

#ifdef PROX_SENSE_DEBUG
#define DPRINTK(args...)	printk(args)
#else
#define DPRINTK(args...)
#endif

/******************************************************************************
 *
 * VCC 1.8 control
 *
 * NOTE:
 *   On EVT3s and later these functions have no effect.
 *
 ******************************************************************************/

void hsdl9100_vcc_enable(void)
{
	board_prox_vcc18_enable();
}

void hsdl9100_vcc_disable(void)
{
	board_prox_vcc18_disable();
}

/******************************************************************************
 *
 * LEDON PWM
 *
 ******************************************************************************/

struct ledon_ctxt {
	struct omap_dm_timer *ledon_timer;
	unsigned int ledon_hz;      /* LEDON pulse frequency */
	unsigned int ledon_duty;    /* LEDON duty cycle in % */
	int          ledon_active;  /* LEDON PWM is on */

	int          enb_gpio;      /* GPIO for ENB pin */
};

int hsdl9100_ledon_pwm_init(struct hsdl9100_platform_data *pdata, void **context)
{
	struct ledon_ctxt *ctx;

	int rc;

	DPRINTK(KERN_INFO "%s.\n", __func__);

	/* Allocate mem for context data structure.
	 */
	ctx = kzalloc(sizeof(struct ledon_ctxt), GFP_KERNEL);
	if (!ctx) {
		return -ENOMEM;
	}

	ctx->ledon_hz     = pdata->ledon_hz;
	ctx->ledon_duty   = pdata->ledon_duty;
	ctx->ledon_active = 0;
	ctx->enb_gpio     = pdata->enable_gpio;

	/* Get the GPIO for the ENB pin.
	 */
	rc = gpio_request(ctx->enb_gpio, "prox enable");
	if (rc < 0) {
		printk(KERN_ERR "%s: Error in requesting gpio: %d...\n",
		       HSDL9100_DRIVER, pdata->enable_gpio);
		goto enable_gpio_request_failed;
	}
	/* Make GPIO output and de-assert (ENB is active low).
	 */
	gpio_direction_output(ctx->enb_gpio, 1);

	/* get the pwm timer for ledon.
	 */
	ctx->ledon_timer = omap_dm_timer_request_specific(pdata->ledon_dm_timer);
	if (!ctx->ledon_timer){
		printk(KERN_ERR "%s: error in requesting omap timer...\n",
		       HSDL9100_DRIVER);
		goto timer_request_failed;
	}

	omap_dm_timer_disable(ctx->ledon_timer);

	*context = ctx;
	return 0;

timer_request_failed:
	gpio_free(ctx->enb_gpio);

enable_gpio_request_failed:
	kfree(ctx);

	return -1;
}

void hsdl9100_ledon_pwm_free(void *context)
{
	struct ledon_ctxt *ctx = (struct ledon_ctxt *) context;

	DPRINTK(KERN_INFO "%s.\n", __func__);
	if (ctx->ledon_timer)
		omap_dm_timer_free(ctx->ledon_timer);

	gpio_free(ctx->enb_gpio);

	kfree(ctx);
}

/*
 * This function must be called with the proximity sensor disabled.
 */
void hsdl9100_ledon_pwm_configure(void *context)
{
	unsigned int tim_clk;
	int pwm_load;
	int pwm_match;

	struct ledon_ctxt *ctx = (struct ledon_ctxt *) context;

	unsigned int ledon_hz   = ctx->ledon_hz;
	unsigned int ledon_duty = ctx->ledon_duty;

	DPRINTK(KERN_INFO "%s.\n", __func__);

	/* Set timer clock source.
	 */
	omap_dm_timer_enable(ctx->ledon_timer);
	omap_dm_timer_set_source(ctx->ledon_timer, OMAP_TIMER_SRC_SYS_CLK);

	/* Calculate load and match values for LEDON pulse frequency and duty
	 * cycle.
	 */
	tim_clk = clk_get_rate(omap_dm_timer_get_fclk(ctx->ledon_timer));

	pwm_load  = -(tim_clk/ledon_hz);
	pwm_match = pwm_load + (((tim_clk / ledon_hz) * ledon_duty) / 100);

	/* The match/load values should be less than the overflow value by min.
	 * 2 units according to specs.
	 */
	if (pwm_match - pwm_load < 2)
		pwm_match = pwm_load + 2;

	/* Turn on the timer clocks before we access timer registers.
	 */
	omap_dm_timer_enable(ctx->ledon_timer);  

	omap_dm_timer_stop     (ctx->ledon_timer);
	/* load value determines cycle time */
	omap_dm_timer_set_load (ctx->ledon_timer, 1, pwm_load);
	/* match value determines duty cycle */
	omap_dm_timer_set_match(ctx->ledon_timer, 1, pwm_match);
        /* default "off" and "pwm" mode */
	omap_dm_timer_set_pwm  (ctx->ledon_timer, 0, 1,
				OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE); 
	/* set counter to overflow immediately */
	omap_dm_timer_write_counter(ctx->ledon_timer, -2);

	/* Turn it off again. We turn it back on once we need it.
	 */
	omap_dm_timer_disable(ctx->ledon_timer);

	DPRINTK(KERN_INFO "LEDON pwm_load:  0x%08x\n", pwm_load);
	DPRINTK(KERN_INFO "LEDON pwm_match: 0x%08x\n", pwm_match);
}

void hsdl9100_ledon_pwm_start(void *context)
{
	struct ledon_ctxt *ctx = (struct ledon_ctxt *) context;

	DPRINTK(KERN_INFO "%s.\n", __func__);

	if (ctx->ledon_active)
		return;

	/* Assert ENB pin (active low).
	 * The part must be enabled for at least 30us before
	 * starting the PWM according to Avago.
	 */
	gpio_set_value(ctx->enb_gpio, 0);

	udelay(40);	

	/* Turn on the LEDON PWM timer.
	 */
	omap_dm_timer_enable(ctx->ledon_timer);

	/* Set counter to 0xFFFFFFFE so we get an overflow event first.
	 */
	omap_dm_timer_set_counter(ctx->ledon_timer, 0xFFFFFFFE);

	omap_dm_timer_start (ctx->ledon_timer);    

	ctx->ledon_active = 1;
}

void hsdl9100_ledon_pwm_stop(void *context)
{
	struct ledon_ctxt *ctx = (struct ledon_ctxt *) context;

	DPRINTK(KERN_INFO "%s.\n", __func__);

	if (!ctx->ledon_active)
		return;

	/* De-assert ENB pin (active low).
	 */
	gpio_set_value(ctx->enb_gpio, 1);

	/* Turn OFF the LEDON PWM timer.
	 */
	omap_dm_timer_enable (ctx->ledon_timer);
	omap_dm_timer_stop   (ctx->ledon_timer);
	omap_dm_timer_disable(ctx->ledon_timer);  

	ctx->ledon_active = 0;
}

int hsdl9100_ledon_pwm_get_duty_cycle(void *context)
{
	struct ledon_ctxt *ctx = (struct ledon_ctxt *) context;
	return ctx->ledon_duty;
}

int hsdl9100_ledon_pwm_get_hz(void *context)
{
	struct ledon_ctxt *ctx = (struct ledon_ctxt *) context;
	return ctx->ledon_hz;
}

void hsdl9100_ledon_pwm_set_duty_cycle(void *context, int duty)
{
	struct ledon_ctxt *ctx = (struct ledon_ctxt *) context;
	ctx->ledon_duty = duty;
}

void hsdl9100_ledon_pwm_set_hz(void *context, int hz)
{
	struct ledon_ctxt *ctx = (struct ledon_ctxt *) context;
	ctx->ledon_hz = hz;
}


