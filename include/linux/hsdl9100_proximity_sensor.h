#ifndef _HSDL9100_PROXIMITY_SENSOR_H
#define _HSDL9100_PROXIMITY_SENSOR_H

#define  HSDL9100_DEVICE   "hsdl9100_proximity"
#define  HSDL9100_DRIVER   "hsdl9100_proximity"

/* HSDL9100 proximity sensor platform data structure */
struct hsdl9100_platform_data {
	char *dev_name;   // device name

	int detect_gpio;
	int enable_gpio;

	int ledon_dm_timer;
	int ledon_hz;
	int ledon_duty;

	int scan_off_time_no_det; /* default time between scans in ms while no
				     object has yet been detected */
	int scan_off_time_det;    /* default time between scans in ms when an
				     object has been detected */
	int scan_on_time;         /* default scan pulse length in ms */
};

/******************************************************************************
 *
 * VCC 1.8 control
 *
 * NOTE:
 *   On EVT3s and later these functions have no effect.
 *
 ******************************************************************************/

void hsdl9100_vcc_enable(void);
void hsdl9100_vcc_disable(void);

/******************************************************************************
 *
 * LEDON PWM API
 *
 ******************************************************************************/

int  hsdl9100_ledon_pwm_init(struct hsdl9100_platform_data *pdata, void **context);
void hsdl9100_ledon_pwm_free(void *context);
void hsdl9100_ledon_pwm_configure(void *context);
void hsdl9100_ledon_pwm_start(void *context);
void hsdl9100_ledon_pwm_stop(void *context);

int  hsdl9100_ledon_pwm_get_duty_cycle(void *context);
int  hsdl9100_ledon_pwm_get_hz(void *context);
void hsdl9100_ledon_pwm_set_duty_cycle(void *context, int duty);
void hsdl9100_ledon_pwm_set_hz(void *context, int hz);

#endif // HSDL9100
