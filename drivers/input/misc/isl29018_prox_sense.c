/*
 * linux/drivers/input/misc/isl29018_prox_sensor.c
 *
 * Driver for the ISL29018 Proximity Sensor.
 *
 * Copyright (C) 2008 Palm, Inc.
 * Author: Dmitry Fink <dmitry.fink@palm.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/isl29018_prox_sense.h>

#include <asm/irq.h>
#include <asm/arch/gpio.h>

#undef PROX_SENSE_DEBUG
//#define PROX_SENSE_DEBUG

#define GPIO_NAME "prox_gpio_irq"

#define POLLED_MODE

// Register offsets
#define ADDR_COMMANDI		(0)
#define ADDR_COMMANDII		(1)
#define ADDR_DATALSB		(2)
#define ADDR_DATAMSB		(3)
#define ADDR_INT_LT_LSB		(4)
#define ADDR_INT_LT_MSB		(5)
#define ADDR_INT_HT_LSB		(6)
#define ADDR_INT_HT_MSB		(7)
#define ADDR_UNDOCUMENTED	(8)

// Operation modes
#define MODE_OFF			(0)
#define MODE_ALS_ONCE		(1)
#define MODE_IR_ONCE		(2)
#define MODE_PROX_ONCE		(3)
#define MODE_ALS_CONT		(5)
#define MODE_IR_CONT		(6)
#define MODE_PROX_CONT		(7)

// Interrupt status om ADDR_COMMANDI, cleared by reading
#define INT_FLAG_MASK		(0x4)

// Maximum ADC values in different width modes
static unsigned int prox_max_adc[] = {
32767,	// 16 bit signed
2047,	// 12 bit signed
127,	// 8 bit signed
15		// 4 bit signed
};

// Integration time in mS of n-BIT ADC
static unsigned int isl_adc_integration_time [] = {
	100, // 16 bit
	6,	// 12 bit
	1,	// 8 bit
	1	// 4 bit
};

#ifndef POLLED_MODE

static irqreturn_t prox_sensor_interrupt(int irq, void *dev_state);

#endif

typedef enum
{
	RANGE_STATE_UNDEFINED = 0,
	RANGE_STATE_IN_RANGE,
	RANGE_STATE_OUT_OF_RANGE
} RANGE_STATE;

/* light sensor state */
#define LIGHTSENSOR_STATE_OFF         0
#define LIGHTSENSOR_STATE_SUSPENDED   1
#define LIGHTSENSOR_STATE_ON          2

struct lightsensor_state {
	struct input_dev * ls_inputdevice;
	int			state;
	int			poll_interval;
	int	   als_inputdev_opencount;
	struct timer_list	poll_timer;
	struct work_struct	poll_work;
};

typedef struct {
	struct input_dev * inputdevice;
	struct isl29018_platform_data* pdata;
	struct i2c_client* client;
	struct {
		// Range detection status
		// undefined, in or out (RANGE_STATE enum)
		unsigned int range_state: 2;

		// Is proximity detection enabled?
		unsigned int enabled: 1;
		unsigned int suspended: 1;
 	};
	struct workqueue_struct* prox_als_workqueue;
	char	prox_als_workqueue_name [32];
	struct work_struct prox_sensor_work;
	struct mutex lock;
#ifdef POLLED_MODE
	struct timer_list scan_timer;
#endif
	int	   prox_sensor_inputdev_opencount;
	// light sensor specific data
	struct lightsensor_state ls_state;
} prox_sensor_state;

#ifdef POLLED_MODE

static inline int scan_interval_for_range (prox_sensor_state *pState)
{
	/* Depending on the detected state we need to change the idle interval
	 * to be short or long. We have a shorter interval for the time when we
	 * have not yet detected an object and a longer interval when we
	 * detected an object.
	 */

	return ( (pState->range_state == RANGE_STATE_IN_RANGE) ? pState->pdata->scan_off_time_det: \
		pState->pdata->scan_off_time_no_det);
}
#endif

/******************************************************************************
 *
 * prox_sensor_get_adc_data()
 *
 ******************************************************************************/
static int prox_sensor_get_adc_data(prox_sensor_state* pState)
{
	int data_lsb, data_msb;

	data_lsb = i2c_smbus_read_byte_data(pState->client, ADDR_DATALSB);
	data_msb = i2c_smbus_read_byte_data(pState->client, ADDR_DATAMSB);

	if ((0 <= data_msb) && (0 <= data_lsb))	{
		return (data_msb << 8) | data_lsb;
	}
	else {
		return -1;
	}
}

#ifndef POLLED_MODE

/******************************************************************************
 *
 * prox_sensor_set_threshold()
 *
 ******************************************************************************/
static int prox_sensor_set_threshold(prox_sensor_state* pState,
												unsigned int lt,
												unsigned int ht)
{
	int rc;
	
	rc = i2c_smbus_write_byte_data(pState->client, ADDR_INT_LT_LSB, lt & 0xff);
	rc |= i2c_smbus_write_byte_data(pState->client, ADDR_INT_LT_MSB, lt >> 8);
	rc |= i2c_smbus_write_byte_data(pState->client, ADDR_INT_HT_LSB, ht & 0xff);
	rc |= i2c_smbus_write_byte_data(pState->client, ADDR_INT_HT_MSB, ht >> 8);

	return rc;	
}
#endif

/******************************************************************************
 *
 * prox_sensor_enable()
 *
 ******************************************************************************/
static int prox_sensor_enable(prox_sensor_state* pState)
{
	int rc = 0;

#ifdef PROX_SENSE_DEBUG
	printk(KERN_ERR "%s: Count: %d Enabled: %d \n", __func__, pState->prox_sensor_inputdev_opencount, pState->enabled);
#endif

	if ( (pState->prox_sensor_inputdev_opencount) && (pState->enabled) ) {

	#ifndef POLLED_MODE
		// Configure the device
		rc = i2c_smbus_write_byte_data(pState->client,
										ADDR_COMMANDII,
										(1 << 7) |	// Dual ADC output
									    (pState->pdata->prox_mod_freq << 6) |
										// LED current amp.
										(pState->pdata->prox_amp << 4) |
										// ADC resolution
										(pState->pdata->prox_width << 2) |
										// Sensitivity
										(pState->pdata->prox_gain));

		// Set device in continous proximity mode
		rc = i2c_smbus_write_byte_data(pState->client,
										ADDR_COMMANDI,
										// Proximity continuous
										(MODE_PROX_CONT << 5) |	
										(pState->pdata->debounce));

		// Wait for the proximity sensor data to settle, which is 1 conversion time
		msleep ( isl_adc_integration_time [pState->pdata->prox_width] );
	#endif
		// Activate work queue for first time to read
		// initial proximity state, report it and configure interrupt
		// and threshold accordingly
		queue_work(pState->prox_als_workqueue, &pState->prox_sensor_work);
	}

	return rc;	
}

/******************************************************************************
 *
 * prox_sensor_disable()
 *
 ******************************************************************************/
static int prox_sensor_disable(prox_sensor_state* pState)
{
	int rc = 0;
	
#ifdef PROX_SENSE_DEBUG
	printk(KERN_ERR "%s: %d \n", __func__, pState->prox_sensor_inputdev_opencount);
#endif
	
#ifndef POLLED_MODE
	/* disable detection irq */
	disable_irq(pState->client->irq);
#endif
	
	rc = i2c_smbus_write_byte_data(pState->client, ADDR_COMMANDI, (MODE_OFF << 5));

#ifdef POLLED_MODE
	del_timer_sync (&pState->scan_timer);
#endif
	return rc;
}


#ifdef POLLED_MODE
/******************************************************************************
 *
 * TIMER handler
 *
 ******************************************************************************/

static void prox_sensor_timer_callback (unsigned long statePtr)
{
	prox_sensor_state *state = (prox_sensor_state *) (statePtr);

	if ( (state->prox_sensor_inputdev_opencount) && (state->enabled) && (!state->suspended) ) {
		queue_work(state->prox_als_workqueue, &state->prox_sensor_work);
	}
}

/******************************************************************************
 *
 *  Worker queue
 *
 ******************************************************************************/
static unsigned int prox_sensor_read_sample (prox_sensor_state* pState)
{
	unsigned int value = 0;

	if (pState->enabled) {

		// Configure the device
		i2c_smbus_write_byte_data(pState->client,
										ADDR_COMMANDII,
										(1 << 7) |	// Dual ADC output
										(pState->pdata->prox_mod_freq << 6) |
										// LED current amp.
										(pState->pdata->prox_amp << 4) |
										// ADC resolution
										(pState->pdata->prox_width << 2) |
										// Sensitivity
										(pState->pdata->prox_gain));

		// Set device in continous proximity mode
		i2c_smbus_write_byte_data(pState->client,
										ADDR_COMMANDI,
										// Proximity once
										(MODE_PROX_ONCE << 5) |	
										(pState->pdata->debounce));

		// Wait for the ADC data to settle (1 conversion time)
		msleep ( isl_adc_integration_time [pState->pdata->prox_width] );
	}

	value = prox_sensor_get_adc_data(pState);

	if (pState->enabled) {
		// Disable the proximity sensor
		i2c_smbus_write_byte_data(pState->client, ADDR_COMMANDI, (MODE_OFF << 5));
	}

	return value;
}

static void prox_sensor_worker(struct work_struct* work)
{
	int value, cmdi_value = 0, rc;
	prox_sensor_state* pState = container_of(work,
									prox_sensor_state, prox_sensor_work);

	mutex_lock(&pState->lock);

	if ( (!pState->enabled) || (pState->suspended) ) {
		mutex_unlock(&pState->lock);
		return;
	}

	// Configure the device
	rc = i2c_smbus_write_byte_data(pState->client,
									ADDR_COMMANDII,
									(1 << 7) |	// Dual ADC output
									(pState->pdata->prox_mod_freq << 6) |
									// LED current amp.
									(pState->pdata->prox_amp << 4) |
									// ADC resolution
									(pState->pdata->prox_width << 2) |
									// Sensitivity
									(pState->pdata->prox_gain));

	// Set device in continous proximity mode
	rc = i2c_smbus_write_byte_data(pState->client,
									ADDR_COMMANDI,
									// Proximity once
									(MODE_PROX_ONCE << 5) |	
									(pState->pdata->debounce));

	// Wait for the ADC data to settle (1 conversion time)
	msleep ( isl_adc_integration_time [pState->pdata->prox_width] );

	value = prox_sensor_get_adc_data(pState);

	// Read from COMMANDI to reset interrupt bit
	cmdi_value = i2c_smbus_read_byte_data(pState->client, ADDR_COMMANDI);

#ifdef PROX_SENSE_DEBUG
	printk(KERN_ERR "prox_sensor_worker hrtimer : ADC=%d\n", value);
#endif

	/*
	 * If ADC reading is more than that max possble ADC for the current resolution
	 * it is a negative number as a of Ambient IR rejection.
	 */
	if (value > prox_max_adc[pState->pdata->prox_width]) {
		value = 0;
	}
	if (value > pState->pdata->threshold) {
		// Detected
		/* report detected state... */
		if (RANGE_STATE_IN_RANGE != pState->range_state) {
			input_event(pState->inputdevice, EV_MSC, MSC_PULSELED, 1);
#ifndef PROX_SENSE_DEBUG
			printk(KERN_INFO "prox_sensor_worker: In range CMDI: %02X CMDII: %02X \n", cmdi_value, \
				i2c_smbus_read_byte_data(pState->client, ADDR_COMMANDII));
#endif
		}
		pState->range_state = RANGE_STATE_IN_RANGE;
	}
	else {
		// Not detected
		/* report detected state... */
		if (RANGE_STATE_OUT_OF_RANGE != pState->range_state) {
			input_event(pState->inputdevice, EV_MSC, MSC_PULSELED, 0);
#ifndef PROX_SENSE_DEBUG
			printk(KERN_INFO "prox_sensor_worker: Out of range CMDI: %02X CMDII: %02X \n", cmdi_value, \
				i2c_smbus_read_byte_data(pState->client, ADDR_COMMANDII));
#endif	
		}
		pState->range_state = RANGE_STATE_OUT_OF_RANGE;
	}

	// Disable the proximity sensor
	i2c_smbus_write_byte_data(pState->client, ADDR_COMMANDI, (MODE_OFF << 5));

	// Set up the timer
	// Factor in the time for the n BIT ADC integration time. This guarantees that the input subsystem
	// receives the notifications, as defined by the scan interval
	mod_timer(&pState->scan_timer, jiffies + msecs_to_jiffies (scan_interval_for_range (pState) ) - \
		msecs_to_jiffies (isl_adc_integration_time [pState->pdata->prox_width] ) - 1);

	mutex_unlock(&pState->lock);
}

#else // #ifdef POLLED_MODE


/******************************************************************************
 *
 *  Worker queue
 *
 ******************************************************************************/
static void prox_sensor_worker(struct work_struct* work)
{
	int value, cmdi_value;
	prox_sensor_state* pState = container_of(work,
									prox_sensor_state, prox_sensor_work);

	mutex_lock(&pState->lock);

	value = prox_sensor_get_adc_data(pState);

#ifdef PROX_SENSE_DEBUG
	printk(KERN_ERR "prox_sensor_worker: ADC=%d\n", value);
#endif

	// Read from COMMANDI to reset interrupt bit
	cmdi_value = i2c_smbus_read_byte_data(pState->client, ADDR_COMMANDI);

	if (!pState->enabled) {
		mutex_unlock(&pState->lock);
		return;
	}

	if (value > pState->pdata->threshold) {
		// Detected
		/* report detected state... */
		if (RANGE_STATE_IN_RANGE != pState->range_state) {
			input_event(pState->inputdevice, EV_MSC, MSC_PULSELED, 1);
#ifndef PROX_SENSE_DEBUG
			printk(KERN_ERR "prox_sensor_worker: in range CMDI: %02X CMDII: %02X \n", cmdi_value, \
				i2c_smbus_read_byte_data(pState->client, ADDR_COMMANDII));
#endif
		}
		pState->range_state = RANGE_STATE_IN_RANGE;
	}
	else {
		// Not detected
		/* report detected state... */
		if (RANGE_STATE_OUT_OF_RANGE != pState->range_state) {
			input_event(pState->inputdevice, EV_MSC, MSC_PULSELED, 0);
#ifndef PROX_SENSE_DEBUG
			printk(KERN_ERR "prox_sensor_worker: out of range CMDI: %02X CMDII: %02X \n", cmdi_value, \
				i2c_smbus_read_byte_data(pState->client, ADDR_COMMANDII));
#endif	
		}
		pState->range_state = RANGE_STATE_OUT_OF_RANGE;
	}

	// Configure next interrupt request based on current reading
	if (RANGE_STATE_IN_RANGE == pState->range_state) {
		// While in range, configure threshold->max as the current range,
		// so that interrupt will be triggered once ADC converges on value
		// less than threshold
		prox_sensor_set_threshold(pState,
									pState->pdata->threshold,
									prox_max_adc[pState->pdata->prox_width]);
	} else {
		// While out of range, configure 0->threshold, interrupt
		// will be generated once ADC reading is more that threshold.
		prox_sensor_set_threshold(pState,
									0,
									pState->pdata->threshold);
	}

#ifdef PROX_SENSE_DEBUG
	printk(KERN_ERR "prox_sensor_worker: re-enable IRQ\n");
#endif

	// Enable interrupt
	enable_irq(pState->client->irq);

	mutex_unlock(&pState->lock);
}

/******************************************************************************
 *
 * prox_sensor_interrupt()
 *
 * Inputs
 *   irq         The IRQ number being serviced. Not used.
 *   dev_state      Device ID provided when IRQ was registered. Not used.
 *
 * Returns
 *   IRQ_HANDLED
 *
 ******************************************************************************/
static irqreturn_t prox_sensor_interrupt(int irq, void *dev_state)
{
	prox_sensor_state* pState = dev_state;
	
#ifdef PROX_SENSE_DEBUG
	printk(KERN_ERR "%s\n", __func__);
#endif

	// Disable interrupt
	disable_irq(pState->client->irq);
	
	// Activate work queue
	queue_work(pState->prox_als_workqueue, &pState->prox_sensor_work);
	
	return IRQ_HANDLED;
}

#endif

/******************************************************************************
 *
 * proximity_show()
 *
 ******************************************************************************/
static ssize_t proximity_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	prox_sensor_state* pState;
	unsigned int adc_value;
	struct input_dev *input_dev = to_input_dev(dev);

	if (!buf)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);

#ifdef POLLED_MODE
	adc_value = prox_sensor_read_sample(pState);
#else
	adc_value = prox_sensor_get_adc_data(pState);
#endif

	/*
	 * If ADC reading is more than that max possble ADC for the current resolution
	 * it is a negative number as a of Ambient IR rejection.
	 */
	if (adc_value > prox_max_adc[pState->pdata->prox_width]) {
		adc_value = 0;
	}
	mutex_unlock(&pState->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", adc_value);
}



/******************************************************************************
 *
 * pm_suspend_show()
 *
 ******************************************************************************/
static ssize_t pm_suspend_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	prox_sensor_state* pState;
	int rc;
	struct input_dev *input_dev = to_input_dev(dev);

	if (!buf)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);

	rc = snprintf(buf, PAGE_SIZE, "%s\n", pState->suspended ? "1" : "0");

	mutex_unlock(&pState->lock);

	return rc;
}

static int prox_sensor_suspend(struct i2c_client *client, pm_message_t pm_state);
static int prox_sensor_resume ( struct i2c_client *client );

/******************************************************************************
 *
 * pm_suspend_store()
 *
 ******************************************************************************/
static ssize_t pm_suspend_store(struct device *dev, struct device_attribute *attr, 
				      const char *buf, size_t count)
{
	prox_sensor_state* pState;
	struct input_dev *input_dev = to_input_dev(dev);
	pm_message_t pm_state;
	
	pm_state.event = 0;

	if (!buf || !count)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	/* suspend or resume based on input buffer[0]*/
	if ('0' == buf[0]) {
		prox_sensor_resume (pState->client);
	} else {
		prox_sensor_suspend (pState->client, pm_state);
	}

	return count;	
}




/******************************************************************************
 *
 * enable_detection_show()
 *
 ******************************************************************************/
static ssize_t enable_detection_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	prox_sensor_state* pState;
	int rc;
	struct input_dev *input_dev = to_input_dev(dev);

	if (!buf)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);

	rc = snprintf(buf, PAGE_SIZE, "%s\n", pState->enabled ? "1" : "0");

	mutex_unlock(&pState->lock);

	return rc;
}

/******************************************************************************
 *
 * prox_sensor_store()
 *
 ******************************************************************************/
static ssize_t enable_detection_store(struct device *dev, struct device_attribute *attr, 
				      const char *buf, size_t count)
{
	prox_sensor_state* pState;
	struct input_dev *input_dev = to_input_dev(dev);

	if (!buf || !count)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);
	
	/* enable or disable detection based on input buffer[0]*/
	if ('0' == buf[0]) {
		if (pState->enabled)
			prox_sensor_disable(pState);
		pState->enabled = 0;
	} else {
		if (!pState->enabled) {
			// Set the flag, since prox_sensor_enable checks for this flag to enable the sensor
			pState->enabled = 1;
			pState->range_state = RANGE_STATE_UNDEFINED;
			prox_sensor_enable(pState);
		}
	}

	mutex_unlock(&pState->lock);
		
	return count;
	
}

/******************************************************************************
 *
 * threshold_show()
 *
 ******************************************************************************/
static ssize_t threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	prox_sensor_state* pState;
	int rc;
	struct input_dev *input_dev = to_input_dev(dev);

	if (!buf)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);

	rc =  snprintf(buf, PAGE_SIZE, "%d\n", pState->pdata->threshold);
	
	mutex_unlock(&pState->lock);

	return rc;
}

/******************************************************************************
 *
 * threshold_store()
 *
 ******************************************************************************/
static ssize_t threshold_store(struct device *dev, struct device_attribute *attr, 
				      const char *buf, size_t count)
{
	prox_sensor_state* pState;
	unsigned int threshold;
	struct input_dev *input_dev = to_input_dev(dev);

	if (!buf || !count)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);
	
	sscanf(buf, "%d", &threshold);

	pState->pdata->threshold = threshold;

#ifndef POLLED_MODE
	if (pState->enabled) {
		prox_sensor_disable(pState);
		prox_sensor_enable(pState);
	}
#endif
	mutex_unlock(&pState->lock);		

	return count;
}

/******************************************************************************
 *
 * prox_amp_show()
 *
 ******************************************************************************/
static ssize_t prox_amp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	prox_sensor_state* pState;
	int rc;
	struct input_dev *input_dev = to_input_dev(dev);

	if (!buf)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);

	rc =  snprintf(buf, PAGE_SIZE, "%d\n", pState->pdata->prox_amp);
	
	mutex_unlock(&pState->lock);

	return rc;
}

/******************************************************************************
 *
 * prox_amp_store()
 *
 ******************************************************************************/
static ssize_t prox_amp_store(struct device *dev, struct device_attribute *attr, 
				      const char *buf, size_t count)
{
	prox_sensor_state* pState;
	unsigned int prox_amp;
	struct input_dev *input_dev = to_input_dev(dev);

	if (!buf || !count)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);
	
	sscanf(buf, "%d", &prox_amp);

	pState->pdata->prox_amp = prox_amp;

	mutex_unlock(&pState->lock);		

	return count;
}

/******************************************************************************
 *
 * prox_width_show()
 *
 ******************************************************************************/
static ssize_t prox_width_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	prox_sensor_state* pState;
	int rc;
	struct input_dev *input_dev = to_input_dev(dev);

	if (!buf)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);

	rc =  snprintf(buf, PAGE_SIZE, "%d\n", pState->pdata->prox_width);
	
	mutex_unlock(&pState->lock);

	return rc;
}

/******************************************************************************
 *
 * prox_width_store()
 *
 ******************************************************************************/
static ssize_t prox_width_store(struct device *dev, struct device_attribute *attr, 
				      const char *buf, size_t count)
{
	prox_sensor_state* pState;
	unsigned int prox_width;
	struct input_dev *input_dev = to_input_dev(dev);

	if (!buf || !count)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);
	
	sscanf(buf, "%d", &prox_width);

	pState->pdata->prox_width = prox_width;

	mutex_unlock(&pState->lock);		

	return count;
}

/******************************************************************************
 *
 * prox_gain_show()
 *
 ******************************************************************************/
static ssize_t prox_gain_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	prox_sensor_state* pState;
	int rc;
	struct input_dev *input_dev = to_input_dev(dev);

	if (!buf)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);

	rc =  snprintf(buf, PAGE_SIZE, "%d\n", pState->pdata->prox_gain);
	
	mutex_unlock(&pState->lock);

	return rc;
}

/******************************************************************************
 *
 * prox_width_store()
 *
 ******************************************************************************/
static ssize_t prox_gain_store(struct device *dev, struct device_attribute *attr, 
				      const char *buf, size_t count)
{
	prox_sensor_state* pState;
	unsigned int prox_gain;
	struct input_dev *input_dev = to_input_dev(dev);

	if (!buf || !count)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);
	
	sscanf(buf, "%d", &prox_gain);

	pState->pdata->prox_gain = prox_gain;

	mutex_unlock(&pState->lock);		

	return count;
}

/******************************************************************************
 *
 * prox_mod_freq_show()
 *
 ******************************************************************************/
static ssize_t prox_mod_freq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	prox_sensor_state* pState;
	int rc;
	struct input_dev *input_dev = to_input_dev(dev);

	if (!buf)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);

	rc =  snprintf(buf, PAGE_SIZE, "%d\n", pState->pdata->prox_mod_freq);
	
	mutex_unlock(&pState->lock);

	return rc;
}

/******************************************************************************
 *
 * prox_mod_freq_store()
 *
 ******************************************************************************/
static ssize_t prox_mod_freq_store(struct device *dev, struct device_attribute *attr, 
				      const char *buf, size_t count)
{
	prox_sensor_state* pState;
	unsigned int prox_mod_freq;
	struct input_dev *input_dev = to_input_dev(dev);

	if (!buf || !count)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);
	
	sscanf(buf, "%d", &prox_mod_freq);

	pState->pdata->prox_mod_freq = prox_mod_freq;

	mutex_unlock(&pState->lock);		

	return count;
}

#ifdef POLLED_MODE
/******************************************************************************
 *
 * scan_off_time_no_det_show()
 *
 ******************************************************************************/
static ssize_t scan_off_time_no_det_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	prox_sensor_state *state;
	struct input_dev *input_dev = to_input_dev(dev);

	state = (prox_sensor_state*)input_get_drvdata(input_dev);

	if (!buf)
		return 0;

	return snprintf(buf, PAGE_SIZE, "%d\n", state->pdata->scan_off_time_no_det);
}

/******************************************************************************
 *
 * scan_off_time_no_det_store()
 *
 ******************************************************************************/
static ssize_t scan_off_time_no_det_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	int value;

	prox_sensor_state *state;
	struct input_dev *input_dev = to_input_dev(dev);

	state = (prox_sensor_state*)input_get_drvdata(input_dev);

	if (!buf || !count)
		return 0;

	sscanf(buf, "%d", &value);

	mutex_lock(&state->lock);

	/* Set the new value if it is valid (>= 50ms).
	 * Otherwise set the default value.
	 */
	if (value >= state->pdata->prox_min_scan_time) {
		state->pdata->scan_off_time_no_det = value;
	} else if (value >= 50) {
		state->pdata->scan_off_time_no_det = value;
		printk("PROX WARNING: Value less than recommended %d ms.\n", state->pdata->prox_min_scan_time);
	} else {
		mutex_unlock(&state->lock);
		printk("PROX ERROR: Value must be >= 50  ms.\n");
		return -EINVAL;
	}

	mutex_unlock(&state->lock);

	return count;
}

/******************************************************************************
 *
 * scan_off_time_det_show()
 *
 ******************************************************************************/
static ssize_t scan_off_time_det_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	prox_sensor_state *state;
	struct input_dev *input_dev = to_input_dev(dev);

	state = (prox_sensor_state*)input_get_drvdata(input_dev);

	if (!buf)
		return 0;

	return snprintf(buf, PAGE_SIZE, "%d\n", state->pdata->scan_off_time_det);
}

/******************************************************************************
 *
 * scan_off_time_det_store()
 *
 ******************************************************************************/
static ssize_t scan_off_time_det_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	int value;
	prox_sensor_state *state;
	struct input_dev *input_dev = to_input_dev(dev);

	state = (prox_sensor_state*)input_get_drvdata(input_dev);
	if (!buf || !count)
		return 0;

	sscanf(buf, "%d", &value);

	mutex_lock(&state->lock);

	/* Set the new value if it is valid (>= 50mS).
	 * Otherwise set the default value.
	 */
	if (value >= state->pdata->prox_min_scan_time) {
		state->pdata->scan_off_time_det = value;
	} else if (value >= 50) {
		state->pdata->scan_off_time_no_det = value;
		printk("PROX WARNING: Value less than recommended %d ms.\n", state->pdata->prox_min_scan_time);
	} else {
		mutex_unlock(&state->lock);
		printk("PROX ERROR: Value must be >= 50 ms.\n");
		return -EINVAL;
	}

	mutex_unlock(&state->lock);

	return count;
}


DEVICE_ATTR(scan_off_time_no_det, S_IRUGO | S_IWUSR, scan_off_time_no_det_show, scan_off_time_no_det_store);
DEVICE_ATTR(scan_off_time_det, S_IRUGO | S_IWUSR, scan_off_time_det_show, scan_off_time_det_store);
#endif

DEVICE_ATTR(pm_suspend, S_IRUGO | S_IWUSR, pm_suspend_show, pm_suspend_store);
DEVICE_ATTR(enable_detection, S_IRUGO | S_IWUSR, enable_detection_show, enable_detection_store);
DEVICE_ATTR(threshold, S_IRUGO | S_IWUSR, threshold_show, threshold_store);
DEVICE_ATTR(prox_gain, S_IRUGO | S_IWUSR, prox_gain_show, prox_gain_store);
DEVICE_ATTR(prox_width, S_IRUGO | S_IWUSR, prox_width_show, prox_width_store);
DEVICE_ATTR(prox_amp, S_IRUGO | S_IWUSR, prox_amp_show, prox_amp_store);
DEVICE_ATTR(prox_mod_freq, S_IRUGO | S_IWUSR, prox_mod_freq_show, prox_mod_freq_store);
DEVICE_ATTR(proximity, S_IRUGO, proximity_show, NULL);

static struct platform_device als_device = {
	.name = "isl-sensor_light",
	.id   = -1,
	.dev  = {
		.platform_data = NULL,
	},
};



/*
 *      Read one sample
 */
static unsigned int ls_read_sample (prox_sensor_state* pState)
{
	unsigned int adc_value = 0;

	// If proximity detection enable, temporary disable it
	if (pState->enabled)
		prox_sensor_disable(pState);

	// Per Intersil, the ADC converter part needs sometime to warm up, so that it
	// can give consistent readings, across different polling periods.
	// The workaround is to do a single ALS_ONCE, so that the ADC converter gets
	// warmed up and then to the actual read.

	i2c_smbus_write_byte_data(pState->client,
									ADDR_COMMANDII,
									// ADC resolution
									(pState->pdata->als_width << 2) |
									// Sensitivity
									(pState->pdata->als_gain));
	i2c_smbus_write_byte_data(pState->client,
								ADDR_COMMANDI,
								(MODE_ALS_ONCE << 5));

	// Wait for the data to settle
	msleep ( isl_adc_integration_time [pState->pdata->als_width] );

	i2c_smbus_write_byte_data(pState->client,
									ADDR_COMMANDII,
									// ADC resolution
									(pState->pdata->als_width << 2) |
									// Sensitivity
									(pState->pdata->als_gain));
	i2c_smbus_write_byte_data(pState->client,
								ADDR_COMMANDI,
								(MODE_ALS_ONCE << 5));

	// Wait for the data to settle
	msleep ( isl_adc_integration_time [pState->pdata->als_width] );

	adc_value = prox_sensor_get_adc_data(pState);

	// Disable
	i2c_smbus_write_byte_data(pState->client, ADDR_COMMANDI, (MODE_OFF << 5));

	// Just to be sure, read COMMANDI to disable interrupt bit
	i2c_smbus_read_byte_data(pState->client, ADDR_COMMANDI);

	// Re-enable proximity detection
	if (pState->enabled)
		prox_sensor_enable(pState);

	return adc_value;
}

/******************************************************************************
 *
 * als_width_show()
 *
 ******************************************************************************/
static ssize_t als_width_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	prox_sensor_state* pState;
	int rc;
	struct input_dev *input_dev = to_input_dev(dev);

	if (!buf)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);

	rc =  snprintf(buf, PAGE_SIZE, "%d\n", pState->pdata->als_width);
	
	mutex_unlock(&pState->lock);

	return rc;
}

/******************************************************************************
 *
 * als_width_store()
 *
 ******************************************************************************/
static ssize_t als_width_store(struct device *dev, struct device_attribute *attr, 
				      const char *buf, size_t count)
{
	prox_sensor_state* pState;
	unsigned int als_width;
	struct input_dev *input_dev = to_input_dev(dev);

	if (!buf || !count)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);
	
	sscanf(buf, "%d", &als_width);

	pState->pdata->als_width = als_width;

	mutex_unlock(&pState->lock);		

	return count;
}

/******************************************************************************
 *
 * als_gain_show()
 *
 ******************************************************************************/
static ssize_t als_gain_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	prox_sensor_state* pState;
	int rc;
	struct input_dev *input_dev = to_input_dev(dev);

	if (!buf)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);

	rc =  snprintf(buf, PAGE_SIZE, "%d\n", pState->pdata->als_gain);
	
	mutex_unlock(&pState->lock);

	return rc;
}

/******************************************************************************
 *
 * als_width_store()
 *
 ******************************************************************************/
static ssize_t als_gain_store(struct device *dev, struct device_attribute *attr, 
				      const char *buf, size_t count)
{
	prox_sensor_state* pState;
	unsigned int als_gain;
	struct input_dev *input_dev = to_input_dev(dev);

	if (!buf || !count)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);
	
	sscanf(buf, "%d", &als_gain);

	pState->pdata->als_gain = als_gain;

	mutex_unlock(&pState->lock);		

	return count;
}


/*
 *   Returns const state string 
 */
static inline const char*
get_state_str(prox_sensor_state* pState)
{
	if (pState->suspended || pState->ls_state.state == LIGHTSENSOR_STATE_SUSPENDED )
		return "suspended";

	if (pState->ls_state.state == LIGHTSENSOR_STATE_ON)
		return "on";

	if (pState->ls_state.state == LIGHTSENSOR_STATE_OFF)
		return "off";

	return "undefined";
}

/******************************************************************************
 *
 * show_result()
 *
 ******************************************************************************/
static ssize_t show_result(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct input_dev *input_dev = to_input_dev(dev);
	prox_sensor_state* pState;
	unsigned int adc_value = 0;

	if (!buf)
		return 0;

	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	mutex_lock(&pState->lock);	
	adc_value = ls_read_sample (pState);
	mutex_unlock(&pState->lock);
	return snprintf(buf, PAGE_SIZE, "%d\n", adc_value);
}

static void als_sensor_start (prox_sensor_state* pState)
{

	if ( (pState->ls_state.als_inputdev_opencount) && (pState->ls_state.state == LIGHTSENSOR_STATE_ON) ) {
		// Restart the timer
		if(!pState->suspended && pState->ls_state.poll_interval > 0 ) {
		
			// Factor in the time for the n BIT ADC integration time. This guarantees that the input subsystem
			// receives als numbers, as defined by the poll_interval
			mod_timer(&pState->ls_state.poll_timer, 
				   jiffies + msecs_to_jiffies ( (pState->ls_state.poll_interval - isl_adc_integration_time [pState->pdata->als_width] - \
				   jiffies_to_msecs (1) ) ) );
		}
	}
}

static void als_sensor_stop (prox_sensor_state* pState)
{
	del_timer_sync(&pState->ls_state.poll_timer);
	cancel_work_sync(&pState->ls_state.poll_work);
}

/*
 *  "state" attribute (on, off, suspended)
 */
static ssize_t
show_state_attr( struct device *dev, struct device_attribute *dev_attr, 
                 char   *buf)
{
	struct input_dev *input_dev = to_input_dev(dev);
	prox_sensor_state* pState;
	pState = (prox_sensor_state*)input_get_drvdata(input_dev);
	return snprintf( buf, PAGE_SIZE, "%s\n", get_state_str(pState)) + 1;
}

static ssize_t 
store_state_attr( struct device *dev, struct device_attribute *dev_attr, 
                  const char *buf, size_t count)
{
	struct input_dev *input_dev = to_input_dev(dev);
	prox_sensor_state* pState;
	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	if (!buf || !count) 
		return -EINVAL; 

	mutex_lock(&pState->lock);
	if (count >= 3 && strncmp( buf, "off", 3) == 0 ) {
		pState->ls_state.state = LIGHTSENSOR_STATE_OFF;
		als_sensor_stop (pState);
	} 
	else if (count >= 9 && strncmp( buf,"suspended", 9) == 0) {
		pState->ls_state.state = LIGHTSENSOR_STATE_SUSPENDED;
		als_sensor_stop (pState);
	}
	else if (count >= 2 && strncmp( buf,"on", 2) == 0 ) {
		pState->ls_state.state = LIGHTSENSOR_STATE_ON;
		als_sensor_start (pState);
	} 
	mutex_unlock(&pState->lock);
	
	return count;
}


static ssize_t 
show_poll_interval( struct device *dev,struct device_attribute *dev_attr, 
                    char *buf)
{
	struct input_dev *input_dev = to_input_dev(dev);
	prox_sensor_state* pState;
	pState = (prox_sensor_state*)input_get_drvdata(input_dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", pState->ls_state.poll_interval);
}

static ssize_t 
store_poll_interval(struct device *dev,struct device_attribute *dev_attr, 
                    const char *buf, size_t count)
{
	struct input_dev *input_dev = to_input_dev(dev);
	int poll_interval;
	prox_sensor_state* pState;
	pState = (prox_sensor_state*)input_get_drvdata(input_dev);

	if (!buf || !count) 
		return -EINVAL; 

	poll_interval =  simple_strtol(buf, NULL, 0);

	mutex_lock(&pState->lock);
	
	/* Set the new value if it is valid (> ADC conversion time + 10 mS).
	 * Otherwise set the default value.
	 */

	if (poll_interval >= pState->pdata->als_min_poll_interval) {
		pState->ls_state.poll_interval =  poll_interval;
	} else {
		mutex_unlock(&pState->lock);
		printk("ALS ERROR: Value must be >= %d mS.\n", pState->pdata->als_min_poll_interval);
		return -EINVAL;
	}
	mutex_unlock(&pState->lock);

	return count;
}

/*
 *   "mode" attribute
 */
static ssize_t
show_mode_attr ( struct device *dev,   struct device_attribute *dev_attr, 
                 char *buf)
{
	// this driver only supports poll mode
	return snprintf(buf, PAGE_SIZE, "poll\n")+1;
}

static ssize_t 
store_mode_attr ( struct device *dev, struct device_attribute *dev_attr, 
                  const char *buf, size_t count)
{
	if (!buf || !count) 
		return -EINVAL; 

	// this driver only supports poll mode
	return count;
}


static DEVICE_ATTR(result, S_IRUGO, show_result, NULL);
static DEVICE_ATTR(poll_interval,  S_IRUGO | S_IWUSR, show_poll_interval,  store_poll_interval);
static DEVICE_ATTR(mode,           S_IRUGO | S_IWUSR, show_mode_attr,      store_mode_attr);
static DEVICE_ATTR(state,          S_IRUGO | S_IWUSR, show_state_attr,     store_state_attr);
static DEVICE_ATTR(als_gain, S_IRUGO | S_IWUSR, als_gain_show, als_gain_store);
static DEVICE_ATTR(als_width, S_IRUGO | S_IWUSR, als_width_show, als_width_store);

static void 
light_sensor_poll_timeout(unsigned long statePtr )
{
	struct lightsensor_state *ls = (struct lightsensor_state *) statePtr;
	prox_sensor_state * state;
	state = container_of(ls, prox_sensor_state, ls_state);

	/* Queue the work */
	queue_work(state->prox_als_workqueue, &ls->poll_work);
}

static void 
light_sensor_poll_result(struct work_struct *work) 
{
	unsigned int ret;
	struct lightsensor_state *ls;
	prox_sensor_state * state;

	ls  = container_of(work, struct lightsensor_state, poll_work);
	state = container_of(ls, prox_sensor_state, ls_state);
	
	mutex_lock(&state->lock);

	// If the device is being suspended, bail out.
	if (state->suspended) {
		mutex_unlock(&state->lock);
		return;
	}
	
	ret = ls_read_sample ( state );
	mutex_unlock(&state->lock);	
	if( ret >= 0 ) {
		input_report_abs(ls->ls_inputdevice, ABS_MISC, ret);
		input_sync(ls->ls_inputdevice);
	}	
	
	if ( (ls->state == LIGHTSENSOR_STATE_ON) && (ls->poll_interval > 0) && (!state->suspended) ) {
		mod_timer(&ls->poll_timer, jiffies + msecs_to_jiffies (ls->poll_interval - ( 2 * (isl_adc_integration_time [state->pdata->als_width] + \
			jiffies_to_msecs (1) ) )));
	}
}


/******************************************************************************
 *
 * als_sensor_open()
 *
 * Called when a client opens the als input device
 ******************************************************************************/
static int als_sensor_open (struct input_dev *idev)
{
	prox_sensor_state* pState = (prox_sensor_state*)input_get_drvdata(idev);
	
	mutex_lock(&pState->lock);

	pState->ls_state.als_inputdev_opencount++;

	if (1 == pState->ls_state.als_inputdev_opencount) {
		als_sensor_start (pState);
	}
	mutex_unlock(&pState->lock);

	return 0;
}

/******************************************************************************
 *
 * als_sensor_close()
 *
 * Called when a client closes the als input device
 ******************************************************************************/
static void als_sensor_close (struct input_dev *idev)
{
	prox_sensor_state* pState = (prox_sensor_state*)input_get_drvdata(idev);
	
	mutex_lock(&pState->lock);
	
	if (1 == pState->ls_state.als_inputdev_opencount) { 
		als_sensor_stop (pState);
	}
	pState->ls_state.als_inputdev_opencount--;

	mutex_unlock(&pState->lock);
}

static int __devinit als_sensor_probe(struct platform_device *pdev)
{
	int ret;
	prox_sensor_state* pState;

	if( pdev->dev.platform_data == NULL ) {
		return (-ENODEV);
	}

	pState = pdev->dev.platform_data;

	pdev->dev.driver_data = pdev->dev.platform_data;

	pState->ls_state.ls_inputdevice = input_allocate_device();
	if (pState->ls_state.ls_inputdevice == NULL) {
		ret = -ENOMEM;
		goto err1;
	}

	/* Default value of poll interval is 1000ms */
	pState->ls_state.poll_interval = 1000;

	pState->ls_state.state = LIGHTSENSOR_STATE_OFF;

	// Init Workque interface  
	INIT_WORK(&pState->ls_state.poll_work, light_sensor_poll_result); 

	pState->ls_state.ls_inputdevice->name = pdev->name;
	pState->ls_state.ls_inputdevice->id.bustype = BUS_VIRTUAL;
	pState->ls_state.ls_inputdevice->id.vendor  = 0x0;
	pState->ls_state.ls_inputdevice->id.product = 0x0;
	pState->ls_state.ls_inputdevice->id.version = 0x100;
	pState->ls_state.ls_inputdevice->dev.parent = &pdev->dev;

	set_bit(EV_ABS, pState->ls_state.ls_inputdevice->evbit); 
	set_bit(ABS_MISC,  pState->ls_state.ls_inputdevice->absbit); 
	ret = input_register_device(pState->ls_state.ls_inputdevice);
	if (ret) {
		goto err2;
	}

	if ((ret = device_create_file(&pState->ls_state.ls_inputdevice->dev, &dev_attr_result))) {
		goto err3;
	}

	if ((ret = device_create_file(&pState->ls_state.ls_inputdevice->dev, &dev_attr_poll_interval))) {
		goto err4;
	}

	if ((ret = device_create_file(&pState->ls_state.ls_inputdevice->dev, &dev_attr_mode))) {
		goto err5;
	}

	if ((ret = device_create_file(&pState->ls_state.ls_inputdevice->dev, &dev_attr_state))) {
		goto err6;
	}

    if ((ret = device_create_file(&pState->ls_state.ls_inputdevice->dev, &dev_attr_als_gain))) {
		goto err7;
	}

	if ((ret = device_create_file(&pState->ls_state.ls_inputdevice->dev, &dev_attr_als_width))) {
		goto err8;
	}

	// Set up a repeating timer
	pState->ls_state.poll_timer.function = light_sensor_poll_timeout;
	pState->ls_state.poll_timer.data = (unsigned long) (&pState->ls_state);

	init_timer_deferrable (&pState->ls_state.poll_timer);

	pState->ls_state.ls_inputdevice->open = als_sensor_open;
	pState->ls_state.ls_inputdevice->close = als_sensor_close;
	pState->ls_state.als_inputdev_opencount = 0;
	
	input_set_drvdata(pState->ls_state.ls_inputdevice, pState);

	goto exit;

err8:
	device_remove_file(&pState->ls_state.ls_inputdevice->dev, &dev_attr_als_gain);
err7:
	device_remove_file(&pState->ls_state.ls_inputdevice->dev, &dev_attr_state);
err6:
	device_remove_file(&pState->ls_state.ls_inputdevice->dev, &dev_attr_mode);
err5:
	device_remove_file(&pState->ls_state.ls_inputdevice->dev, &dev_attr_poll_interval);
err4:
	device_remove_file(&pState->ls_state.ls_inputdevice->dev, &dev_attr_result);
err3:
	input_unregister_device(pState->ls_state.ls_inputdevice);
err2:
	input_free_device(pState->ls_state.ls_inputdevice);
err1:
	return -1;
exit:
	return 0;
}

static int __devexit als_sensor_remove(struct platform_device *pdev)
{	
	prox_sensor_state* pState;

	if( pdev->dev.platform_data == NULL ) {
		return (-ENODEV);
	}

	pState = pdev->dev.platform_data;

	del_timer_sync(&pState->ls_state.poll_timer);


	cancel_work_sync(&pState->ls_state.poll_work);

	device_remove_file(&pState->ls_state.ls_inputdevice->dev, &dev_attr_result);
	device_remove_file(&pState->ls_state.ls_inputdevice->dev, &dev_attr_poll_interval);
	device_remove_file(&pState->ls_state.ls_inputdevice->dev, &dev_attr_mode);
	device_remove_file(&pState->ls_state.ls_inputdevice->dev, &dev_attr_state);
	device_remove_file(&pState->ls_state.ls_inputdevice->dev, &dev_attr_als_gain);
	device_remove_file(&pState->ls_state.ls_inputdevice->dev, &dev_attr_als_width);

	input_free_device(pState->ls_state.ls_inputdevice);

	pdev->dev.driver_data = NULL; 

	return 0;
}

static struct platform_driver als_driver = {
	.probe		= als_sensor_probe,
	.remove		= __devexit_p(als_sensor_remove),
	.driver		= {
		.name	= "isl-sensor_light",
	},
};

/******************************************************************************
 *
 * als_sensor_init()
 *
 *   0 on success, or non-zero otherwise.
 *
 ******************************************************************************/
static int als_sensor_init(prox_sensor_state* pState)
{
	int err;
	
	err = platform_driver_register(&als_driver);

	if (unlikely(err < 0))
		return err;

	als_device.dev.platform_data = pState;
	err = platform_device_register(&als_device);

	if (err) {
		platform_driver_unregister(&als_driver);
		return err;
	}
		
  return 0;
}

/******************************************************************************
 *
 * als_sensor_exit()
 *
 ******************************************************************************/
static void  als_sensor_exit(void)
{	
	platform_device_unregister(&als_device);
	platform_driver_unregister(&als_driver);
}

/******************************************************************************
 *
 * prox_sensor_open()
 *
 * Called when a client opens the proximity input device
 ******************************************************************************/
static int prox_sensor_open (struct input_dev *idev)
{
	prox_sensor_state* pState = (prox_sensor_state*)input_get_drvdata(idev);

	mutex_lock(&pState->lock);
	
	pState->prox_sensor_inputdev_opencount++;

	if (1 == pState->prox_sensor_inputdev_opencount) {
		// If the proximity sensor is set as enabled, enable the proximity sensor
		if (pState->enabled) {
			pState->range_state = RANGE_STATE_UNDEFINED;
			prox_sensor_enable(pState);
		}
	}

	mutex_unlock(&pState->lock);

	return 0;
}

/******************************************************************************
 *
 * prox_sensor_close()
 *
 * Called when a client closes the proximity input device
 ******************************************************************************/
static void prox_sensor_close (struct input_dev *idev)
{
	prox_sensor_state* pState = (prox_sensor_state*)input_get_drvdata(idev);
	
	mutex_lock(&pState->lock);

	if (1 == pState->prox_sensor_inputdev_opencount) {
		if (pState->enabled) {
			prox_sensor_disable(pState);
		}
	}
	pState->prox_sensor_inputdev_opencount--;

	mutex_unlock(&pState->lock);
}

/******************************************************************************
 *
 * prox_sensor_probe()
 *
 * Called by driver model to initialize device
 *
 * Returns
 *   0 on success, or non-zero otherwise.
 * 
 ******************************************************************************/
static int __init prox_sensor_probe(struct i2c_client *client)
{
	int rc;
	/* local state */
	prox_sensor_state* pState;

#ifdef PROX_SENSE_DEBUG
	printk("%s.\n", __func__);
#endif

	pState = kzalloc(sizeof(prox_sensor_state), GFP_KERNEL);
	if (!pState) {
		printk(KERN_ERR "isl29018: Out Of Memory!\n");
		return -ENOMEM;
	}

	pState->pdata = (struct isl29018_platform_data*)client->dev.platform_data;
	pState->client = client;
	
	/* init local state */
	mutex_init(&pState->lock);
	pState->range_state = RANGE_STATE_UNDEFINED;
	pState->enabled = 0;
	pState->suspended = 0;

	/* allocate and configure input device */
	pState->inputdevice = input_allocate_device();
	if (!pState->inputdevice) {
		goto err1;
	}

#ifdef POLLED_MODE
	// Set up a repeating timer
	pState->scan_timer.function = prox_sensor_timer_callback;
	pState->scan_timer.data = (unsigned long) (pState);
	init_timer_deferrable (&pState->scan_timer);
#endif

	pState->inputdevice->name = ISL29018_DEVICE;
	set_bit(EV_MSC,       pState->inputdevice->evbit  );
	set_bit(MSC_PULSELED, pState->inputdevice->mscbit );

	pState->inputdevice->open = prox_sensor_open;
	pState->inputdevice->close = prox_sensor_close;
	pState->prox_sensor_inputdev_opencount = 0;

	rc = input_register_device(pState->inputdevice);
	if (rc < 0) {
		printk(KERN_ERR "isl29018: Error in registering input device...\n");
		goto err2;    
	}

#ifndef POLLED_MODE
	/* setup gpio */
	rc = gpio_request(pState->pdata->gpio, GPIO_NAME);
	if (rc < 0) {
		printk(KERN_ERR "isl29018: Error in requesting gpio: %d...\n", 
		       pState->pdata->gpio);
		goto err3;    
	}
	gpio_direction_input(pState->pdata->gpio);
#endif

	/* create device sysfs attributes */
	rc = device_create_file(&pState->inputdevice->dev, &dev_attr_enable_detection);
	if (rc != 0) {
		printk(KERN_ERR "isl29018: Error in creating device sysfs entries...\n");
		goto err4;
	}
	

	/* create device sysfs attributes */
	rc = device_create_file(&pState->inputdevice->dev, &dev_attr_proximity);
	if (rc != 0) {
		printk(KERN_ERR "isl29018: Error in creating device sysfs entries...\n");
		goto err5;
	}

	/* create device sysfs attributes */
	rc = device_create_file(&pState->inputdevice->dev, &dev_attr_threshold);
	if (rc != 0) {
		printk(KERN_ERR "isl29018: Error in creating device sysfs entries...\n");
		goto err6;
	}

    /* create device sysfs attributes */
	rc = device_create_file(&pState->inputdevice->dev, &dev_attr_prox_gain);
	if (rc != 0) {
		printk(KERN_ERR "isl29018: Error in creating device sysfs entries...\n");
		goto err6a;
	}

    /* create device sysfs attributes */
	rc = device_create_file(&pState->inputdevice->dev, &dev_attr_prox_width);
	if (rc != 0) {
		printk(KERN_ERR "isl29018: Error in creating device sysfs entries...\n");
		goto err6b;
	}
	
    /* create device sysfs attributes */
	rc = device_create_file(&pState->inputdevice->dev, &dev_attr_prox_amp);
	if (rc != 0) {
		printk(KERN_ERR "isl29018: Error in creating device sysfs entries...\n");
		goto err6c;
	}

    /* create device sysfs attributes */
	rc = device_create_file(&pState->inputdevice->dev, &dev_attr_prox_mod_freq);
	if (rc != 0) {
		printk(KERN_ERR "isl29018: Error in creating device sysfs entries...\n");
		goto err6d;
	}

	/* create device sysfs attributes */
	rc = device_create_file(&pState->inputdevice->dev, &dev_attr_pm_suspend);
	if (rc != 0) {
		printk(KERN_ERR "isl29018: Error in creating device sysfs entries...\n");
		goto err7;
	}

#ifdef POLLED_MODE
	/* create device sysfs attributes */
	rc = device_create_file(&pState->inputdevice->dev, &dev_attr_scan_off_time_det);
	if (rc != 0) {
		printk(KERN_ERR "isl29018: Error in creating device sysfs entries...\n");
		goto err8;
	}
	/* create device sysfs attributes */
	rc = device_create_file(&pState->inputdevice->dev, &dev_attr_scan_off_time_no_det);
	if (rc != 0) {
		printk(KERN_ERR "isl29018: Error in creating device sysfs entries...\n");
		goto err8a;
	}
#endif

	/* create ambient light sensor device */
	rc = als_sensor_init(pState);
	if (rc != 0) {
		printk(KERN_ERR "isl29018: Error in ambient light sensor device...\n");
#ifdef POLLED_MODE
		goto err8b;
#else
		goto err8;
#endif
	}
	
	// Create the work queue
	snprintf(pState->prox_als_workqueue_name, sizeof(pState->prox_als_workqueue_name), "prox_als_workqueue");
	pState->prox_als_workqueue = create_singlethread_workqueue (pState->prox_als_workqueue_name);

	INIT_WORK(&pState->prox_sensor_work, prox_sensor_worker);
	
	// As part of ISL29018 sensor initialization, reset the COMMANDI register
   	i2c_smbus_write_byte_data(pState->client, ADDR_COMMANDI, 0x00);
   	i2c_smbus_write_byte_data(pState->client, ADDR_COMMANDII, 0x00);

	// Reset the undocumented register as well, as this register gets set to some unexpected values
	// by the sensor itself (which makes the sensor non-functional).
	// Has been approved by Intersil
	i2c_smbus_write_byte_data(pState->client, ADDR_UNDOCUMENTED, 0x00);

	// Read from COMMANDI to reset interrupt bit
	i2c_smbus_read_byte_data(pState->client, ADDR_COMMANDI);

#ifndef POLLED_MODE
	rc = request_irq(client->irq, prox_sensor_interrupt,
						IRQF_TRIGGER_LOW, ISL29018_DEVICE,
						(void*)pState);
	if( rc < 0 ) {
		printk(KERN_ERR "isl29018: Error in requesting irq...\n");
		goto err9;
	}
	/* disable till we enable detection */
	disable_irq(pState->client->irq);
#endif




	/* attach driver data */
	i2c_set_clientdata(client, pState);
	input_set_drvdata(pState->inputdevice, pState);
    

	printk("ISL29018 Proximity Sensor driver initialized...\n");
  
	return 0;

#ifndef POLLED_MODE
err9:
	als_sensor_exit();
#else
err8b:
	device_remove_file(&pState->inputdevice->dev, &dev_attr_scan_off_time_no_det);
err8a:
	device_remove_file(&pState->inputdevice->dev, &dev_attr_scan_off_time_det);
#endif

err8:
	device_remove_file(&pState->inputdevice->dev, &dev_attr_pm_suspend);
err7:
	device_remove_file(&pState->inputdevice->dev, &dev_attr_prox_mod_freq);
err6d:
	device_remove_file(&pState->inputdevice->dev, &dev_attr_prox_amp);
err6c:
	device_remove_file(&pState->inputdevice->dev, &dev_attr_prox_width);
err6b:
	device_remove_file(&pState->inputdevice->dev, &dev_attr_prox_gain);
err6a:
	device_remove_file(&pState->inputdevice->dev, &dev_attr_threshold);
err6:
	device_remove_file(&pState->inputdevice->dev, &dev_attr_proximity);
err5:
	device_remove_file(&pState->inputdevice->dev, &dev_attr_enable_detection);
err4:
	gpio_free(pState->pdata->gpio);
#ifndef POLLED_MODE
err3:
#endif
	input_unregister_device(pState->inputdevice);
err2:
	input_free_device (pState->inputdevice);
err1:
	kfree(pState);
	
	printk( KERN_INFO "Failed to initialize ISL29018 Proximity Sensor driver\n");
	return -ENODEV;    
}

/******************************************************************************
 *
 * prox_sensor_remove()
 *
 ******************************************************************************/
static int prox_sensor_remove(struct i2c_client *client)
{
	prox_sensor_state* pState;

	pState = (prox_sensor_state*)i2c_get_clientdata(client);

	if (pState->enabled) {
		prox_sensor_disable(pState);
	}

#ifndef POLLED_MODE
	free_irq(pState->client->irq, pState);
#endif

	als_sensor_exit();

	device_remove_file(&pState->inputdevice->dev, &dev_attr_pm_suspend);
	device_remove_file(&pState->inputdevice->dev, &dev_attr_threshold);
	device_remove_file(&pState->inputdevice->dev, &dev_attr_proximity);
	device_remove_file(&pState->inputdevice->dev, &dev_attr_enable_detection);
	device_remove_file(&pState->inputdevice->dev, &dev_attr_prox_gain);
	device_remove_file(&pState->inputdevice->dev, &dev_attr_prox_width);
	device_remove_file(&pState->inputdevice->dev, &dev_attr_prox_amp);
	device_remove_file(&pState->inputdevice->dev, &dev_attr_prox_mod_freq);

#ifdef POLLED_MODE
	device_remove_file(&pState->inputdevice->dev, &dev_attr_scan_off_time_no_det);
	device_remove_file(&pState->inputdevice->dev, &dev_attr_scan_off_time_det);
#else
	gpio_free(pState->pdata->gpio);
#endif

	destroy_workqueue(pState->prox_als_workqueue);

	input_unregister_device(pState->inputdevice);
	input_free_device (pState->inputdevice);

	kfree(client->dev.driver_data);
	client->dev.driver_data = NULL;
	
	return 0;
}

/******************************************************************************
 *
 * prox_sensor_suspend()
 * 
 ******************************************************************************/
#ifdef CONFIG_PM
static int prox_sensor_suspend(struct i2c_client *client, pm_message_t pm_state)
{
	int rc = 0;
	prox_sensor_state* pState;

#ifdef PROX_SENSE_DEBUG
	printk(KERN_ERR "%s.\n", __func__);
#endif

	pState = (prox_sensor_state*)i2c_get_clientdata(client);

	mutex_lock(&pState->lock);
	
	// already suspended? 
	if (pState->suspended) {
		goto err0; // Y: return
	}
	
	// already disabled?
	if (pState->enabled) {
		prox_sensor_disable(pState); // N: disable
	}

	// cancel light sensor polling
	del_timer(&pState->ls_state.poll_timer);
	cancel_work_sync(&pState->ls_state.poll_work);

	// mark as suspended
	pState->suspended = 1;
	
err0:  
	mutex_unlock(&pState->lock);
	return rc;
}

/******************************************************************************
 *
 * prox_sensor_resume()
 * 
 ******************************************************************************/
static int prox_sensor_resume ( struct i2c_client *client )
{
	int rc = 0;
	prox_sensor_state* pState;
	
#ifdef PROX_SENSE_DEBUG
	printk(KERN_ERR "%s.\n", __func__);
#endif

	pState = (prox_sensor_state*)i2c_get_clientdata(client);

	mutex_lock(&pState->lock);
		
	// not suspended? 
	if (!pState->suspended) {
		goto err0; // Y: return
	}
	

	// As part of ISL29018 sensor initialization, reset the COMMANDI register
   	i2c_smbus_write_byte_data(pState->client, ADDR_COMMANDI, 0x00);
   	i2c_smbus_write_byte_data(pState->client, ADDR_COMMANDII, 0x00);

	// Reset the undocumented register as well, as this register gets set to some unexpected values
	// by the sensor itself.
	i2c_smbus_write_byte_data(pState->client, ADDR_UNDOCUMENTED, 0x00);

	// enabled before suspend?
	if (pState->enabled) {
		rc = prox_sensor_enable(pState); // Y: enable
	}

	// restart the light sensor polling
	if (pState->ls_state.state == LIGHTSENSOR_STATE_ON && pState->ls_state.poll_interval > 0) {
		mod_timer(&pState->ls_state.poll_timer, 
		           jiffies + msecs_to_jiffies(pState->ls_state.poll_interval - ( 2 * (isl_adc_integration_time [pState->pdata->als_width] + \
				   jiffies_to_msecs (1) )) ));
	}
	
	// mark as resumed
	pState->suspended = 0;
	
err0:
	mutex_unlock(&pState->lock);
	return 0;
}
#else
#define prox_sensor_suspend  NULL
#define prox_sensor_resume   NULL
#endif  /* CONFIG_PM */


static struct i2c_driver prox_sensor_driver = {
	.probe 		= prox_sensor_probe,
	.remove 	= prox_sensor_remove,
	.suspend 	= prox_sensor_suspend,
	.resume 	= prox_sensor_resume,
	.driver = {
		.name	= ISL29018_DRIVER,
	},
};

/******************************************************************************
 *
 * prox_sensor_init()
 *
 *   0 on success, or non-zero otherwise.
 *
 ******************************************************************************/
static int __init prox_sensor_init(void)
{
	int ret;
	
	ret = i2c_add_driver(&prox_sensor_driver);
	if (ret != 0) {
		return -ENODEV;
	}
		
  return 0;
}

/******************************************************************************
 *
 * prox_sensor_exit()
 *
 ******************************************************************************/
static void  __exit prox_sensor_exit(void)
{	
	i2c_del_driver(&prox_sensor_driver);
}

module_init(prox_sensor_init);
module_exit(prox_sensor_exit);

MODULE_DESCRIPTION("ISL29018 Proximity Sensor Driver");
MODULE_AUTHOR("Dmitry Fink <dmitry.fink@palm.com>");
MODULE_LICENSE("GPL");


