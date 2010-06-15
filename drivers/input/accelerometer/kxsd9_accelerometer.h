#ifndef __KXSD9_ACCELEROMETER_H__ 
#define __KXSD9_ACCELEROMETER_H__  

#ifdef  MODDEBUG
#define PDBG(args...)   printk(args)
#else
#define PDBG(args...)   
#endif

/* Registers */
#define XOUT_H_READ_REG				0x00
#define RESET_WRITE_REG				0x0A   
#define CTRL_REGC	 				0x0C
#define CTRL_REGB   				0X0D
#define CTRL_REGA     				0X0E

/* Input ids */
#define ACCELEROMETER_VENDOR_ID  	0x0001
#define ACCELEROMETER_PRODUCT_ID  	0x0001
#define ACCELEROMETER_VERSION_ID  	0x0100

/* Register settings */
#define RESET_WRITE_VAL 			0xCA
#define CTRL_REGB_VAL 				0xC0
#define CTRL_REGC_VAL 				0xEB

/* Test related */
#define ACCELEROMETER_REG_TEST 			1
#define ACCELEROMETER_INTERRUPT_TEST 	2
#define ACCELEROMETER_POLLING_TEST 		3

#define TEST_PASS				0
#define TEST_FAIL				1
#define INVALID_TEST			2

/* sensing range of accelerometer xyz */

#define ACCEL_MIN_ABS_X		(-2 * SENSITIVITY_UNITS_PER_ACCELERATION_1G)
#define ACCEL_MAX_ABS_X		(2 * SENSITIVITY_UNITS_PER_ACCELERATION_1G)
#define ACCEL_MIN_ABS_Y		(-2 * SENSITIVITY_UNITS_PER_ACCELERATION_1G)
#define ACCEL_MAX_ABS_Y		(2 * SENSITIVITY_UNITS_PER_ACCELERATION_1G)
#define ACCEL_MIN_ABS_Z		(-2 * SENSITIVITY_UNITS_PER_ACCELERATION_1G)
#define ACCEL_MAX_ABS_Z		(2 * SENSITIVITY_UNITS_PER_ACCELERATION_1G)

/*  */
#define MOTION_INTERRUPT_BIT 			0x04
#define ACCELEROMETER_ENABLE_BIT 		0x40

/*  */
#define POLL_INTERVAL				250

#define MODE_OFF                 		0
#define MODE_POLL_ALL              		1
#define MODE_POLL_AXIS             		2
#define MODE_INTERRUPT           		3

/* */
#define CLEAR_MOTION_WAKE_UP_THRESHOLD	0x13
#define CLEAR_FILTER_FRQUENCY			0xE0

/* */
#define ENABLE_MOTION_INTERRUPT			1
#define DISABLE_MOTION_INTERRUPT		0

/* */
#define TIMER_ENABLED					1
#define TIMER_DISABLED					0

#endif /* __KXSD9_ACCELEROMETER_H__  */ 
