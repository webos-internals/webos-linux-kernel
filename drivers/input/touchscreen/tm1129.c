/*
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/wait.h>
#include <linux/bitops.h>
#include <linux/ctype.h>
#include <linux/spi/spi.h>
#include <linux/spi/tm1129.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>

#include <asm/arch/gpio.h>

#undef  MODDEBUG
//#define MODDEBUG

#ifdef  MODDEBUG
#define PDBG(args...)   printk(args)
#else
#define PDBG(args...)   
#endif

#undef  PRINT_EVENTS
// #define PRINT_EVENTS


/*
 * Constants
 */
#define  TSC_NAME       "tm1129"
#define  DEVICE         TSC_NAME
#define  DRIVER         TSC_NAME
#define  DRIVER_DESC    TSC_NAME" touchscreen driver"

#define  TSC_MAX_ABS_X	3645  
#define  TSC_MAX_ABS_Y  5364
#define  TSC_MAX_ABS_Z  255
#define  TSC_MAX_ABS_W  15
#define  TSC_MAX_ABS_F  3
#define  TSC_NUM_CAP_BUTTONS	5
#define  TSC_BUTTON_X_RANGE	(TSC_MAX_ABS_X/TSC_NUM_CAP_BUTTONS)

/*
 * Definitions and Prototypes
 */
typedef enum {TSC_PEN_UP, TSC_PEN_DOWN} TSC_PEN_STATE;

/* Core navi Capacitive Z button structure */
typedef struct cap_button {
    unsigned char norm_z;
    unsigned char max_z;
    unsigned char min_z;
} cap_button_t;

typedef struct tsc_drv_data {
    struct spi_device *spidev;
    struct input_dev  *indev;
    struct tm1129_platform_data *pdata;
    TSC_PEN_STATE      pen_state;
    struct timer_list  pen_timer;
    struct work_struct workq;
    int                suspended;
    DECLARE_BITMAP(gpio_irq_disabled, 1);
    struct attribute_group attrs;
    cap_button_t       cap_btn[TSC_NUM_CAP_BUTTONS]; 
} tsc_drv_data_t;

/* Parameters for transforming X,Y points; defaults leave X,Y unchanged */
static s32 tformX[] = { 4096,     0,  0 };
static s32 tformY[] = {    0,  4096,  0 };
static u32 tformDiv =   4096;     // transform divider

static int tm1129_reg_read ( struct tsc_drv_data *dev, u16 reg, u8 *data, u8 *status);
static int tm1129_reg_write ( struct tsc_drv_data *dev, u16 reg, u8 data, u8 *status);
static int tm1129_reg_read_six( struct tsc_drv_data *dev, u16 reg, u8 *data, u8 *status);

/*
 *  Enable IRQ
 */
static void 
tm1129_enable_irq( struct tsc_drv_data *dev )
{
    if( test_and_clear_bit(0, dev->gpio_irq_disabled)) {
        enable_irq ( dev->spidev->irq );
        /* We are only interested in the ABS position. */
        tm1129_reg_write (dev, INTERRUPT_ENABLE,
                          (INTERRUPT_ENABLE_ABS_INT_EN | \
                           INTERRUPT_ENABLE_BTN_INT_EN), NULL);
    }
}

/*
 *  Disable IRQ
 */
static void 
tm1129_disable_irq ( struct tsc_drv_data *dev )
{
    if(!test_and_set_bit(0, dev->gpio_irq_disabled)) {
        disable_irq( dev->spidev->irq  );
        tm1129_reg_write (dev, INTERRUPT_ENABLE, 0, NULL);
    }
}

/*
 *  tm1129 sync read six consecutive register routine
 */
static int 
tm1129_reg_read_six(struct tsc_drv_data *dev, u16 reg, u8 *data, u8 *status)
{   
    struct spi_message  m;
    struct spi_transfer t[8];
    u32 err;
    u8 i, tx[8], rx[8];

    memset(&t, 0, sizeof(t));
    spi_message_init(&m);

    /* 16 bit wide addess with READ_BIT */
    tx[0] = SPI_READ_BIT | ((reg >> 8) & 0xff);
    tx[1] = reg & 0xff; 

    for (i = 0; i < 8; i++) {
        t[i].bits_per_word = 8;
        t[i].tx_buf = &tx[i];
        t[i].rx_buf = &rx[i];
        t[i].len    = 1;
        /* Due to the deficiency in Synaptics FW controller,
         * we need 200 usecs delay for first two bytes of
         * read command.
         */
        if (i < 2) {
            t[i].delay_usecs = 200;
        } else {
            t[i].delay_usecs = 100;
        }
        spi_message_add_tail(&t[i], &m);
    }
    err = spi_sync ( dev->spidev, &m);

    /* The second returned byte is the status byte. */
    if (status != NULL) *status = rx[1];

    /* Real read data starts at byte 3 */
    memcpy(data, &rx[2], 6);

    return err;
}

/*
 *  tm1129 sync read register routine
 */
static int 
tm1129_reg_read ( struct tsc_drv_data *dev, u16 reg, u8 *data, u8 *status)
{   
    struct spi_message  m;
    struct spi_transfer t[3];
    u32 err;
    u8 i, tx[3], rx[3];

    memset(&t, 0, sizeof(t));
    spi_message_init(&m);

    /* 16 bit wide address with READ_BIT */
    tx[0] = SPI_READ_BIT | ((reg >> 8) & 0xff);
    tx[1] = reg & 0xff; 

    for (i = 0; i < 3; i++) {
        t[i].bits_per_word = 8;
        t[i].tx_buf = &tx[i];
        t[i].rx_buf = &rx[i];
        t[i].len    = 1;
        /* Due to the deficiency in Synaptics FW controller,
         * we need 200 usecs delay for first two bytes of
         * read command.
         */
        if (i < 2) {
            t[i].delay_usecs = 200;
        } else {
            t[i].delay_usecs = 100;
        }
        spi_message_add_tail(&t[i], &m);
    }
    err = spi_sync ( dev->spidev, &m);

    /* The second returned byte is the status byte. */
    if (status != NULL) *status = rx[1];

    /* Real read data starts at byte 3 */
    *data = rx[2];

    return err;
}


/*
 *  tm1129 sync write register routine
 */
static int 
tm1129_reg_write ( struct tsc_drv_data *dev, u16 reg, u8 data, u8 *status)
{
    struct spi_message  m;
    struct spi_transfer t[3];
    u32 err;
    u8 i, tx[3], rx[3];

    memset(&t, 0, sizeof(t));
    spi_message_init(&m);

    /* 16 bit wide address */
    tx[0] = (reg >> 8) & 0xff;
    tx[1] = reg & 0xff; 

    /* Data we want to write */
    tx[2] = data; 

    for (i = 0; i < 3; i++) {
        t[i].bits_per_word = 8;
        t[i].tx_buf = &tx[i];
        t[i].rx_buf = &rx[i];
        t[i].len    = 1;
        /* Due to the deficiency in Synaptics FW controller,
         * we need 200 usecs delay for first two bytes of
         * write command.
         */
        if (i < 2) {
            t[i].delay_usecs = 200;
        } else {
            t[i].delay_usecs = 100;
        }
        spi_message_add_tail(&t[i], &m);
    }
    err = spi_sync ( dev->spidev, &m);
    
    /* The second returned byte is the status byte. */
    if (status != NULL) *status = rx[1];

    return err;
}

/*
 *  X scroller implementation from five Z buttons
 *  in the secondary core navi area. 
 */
static int
x_scroller(cap_button_t *cap_btn, unsigned char *data)
{
    unsigned char count = 0;
    unsigned char max[3] = {0, 0, 0};
    unsigned char button[3] = {0, 0, 0};
    int i, x = 0;

    /* Perform low & high filter then normalize the Z data. */
    for (i = 0; i < TSC_NUM_CAP_BUTTONS; i++) {

        /* Filter out low and high Z value. */
        if (data[i] < cap_btn[i].min_z) {
            data[i] = cap_btn[i].min_z;
        } else if (data[i] > cap_btn[i].max_z) {
            data[i] = cap_btn[i].max_z;
        }

        /* Normalize to 0 to 100 scale. */
        cap_btn[i].norm_z = 100 * (data[i] - cap_btn[i].min_z) /
                            (cap_btn[i].max_z - cap_btn[i].min_z);

        /* Reduce the noise again on normalized data. */
        if (cap_btn[i].norm_z < 10) {
            cap_btn[i].norm_z = 0;
        }
    }

     /* Find out the top three highest norm_z buttons.
      * MAX array holds Z values and Button array holds the button ID.
      */
    for (i = 0; i < TSC_NUM_CAP_BUTTONS; i++) {

        /* The new Z button has the highest normalized Z value. */
        if (max[0] < cap_btn[i].norm_z) {
            max[2] = max[1];
            max[1] = max[0];
            max[0] = cap_btn[i].norm_z; 
            button[2] = button[1];
            button[1] = button[0];
            button[0] = i;

        /* The new Z button has the second highest normalized Z value. */
        } else if (max[1] < cap_btn[i].norm_z) {
            max[2] = max[1];
            max[1] = cap_btn[i].norm_z;
            button[2] = button[1];
            button[1] = i;

        /* The new Z button has the third highest normalized Z value. */
        } else if (max[2] < cap_btn[i].norm_z) {
            max[2] = cap_btn[i].norm_z; 
            button[2] = i;
        }

    }

    /* Find out the number of buttons with qualified Z values. */
    for (i = 0; i < 3; i++) {
        if (max[i] != 0) {
            count++;
        }
    }

    /* Reduce three count to two. */
    if (count == 3) {
        max[1] = max[1] - max[2];
        count--;
    }

    /* One button. */
    if (count == 1) {

        /* It must be either first or the last button. */
        if ((button[0] != 0) && (button[0] != 4)) {
            return -1;
        }

        /* Calculate X offset from the outer edge. */
        x = (TSC_BUTTON_X_RANGE / 2) * max[0] / 100; 

        if (button[0] == 4) {
            x = TSC_MAX_ABS_X - x; 
        }

    /* Two buttons. */
    } else if (count == 2) {

        x = 0;

        /* Two buttons must be next to each other. */
        if (((button[0] - button[1]) != 1) &&
            ((button[1] - button[0]) != 1)) {
            return -1;
        }

        /* Right button returned higher or equal
         * normalized Z value than the left button.
         */
        if (button[0] > button[1]) {
            /* Find the X value in the middle of two buttons. */
            for (i = 0; i < button[0]; i++) {
                x = x + TSC_BUTTON_X_RANGE; 
            }

            /* Calculate the weighted distance. */
            x = x + (TSC_BUTTON_X_RANGE / 2) * (max[0] - max[1]) / 100; 

        /* Left button returned higher or equal
         * normalized Z value than the right button.
         */ 
        } else { 
            /* Find the X value in the middle of two buttons. */
            for (i = 0; i < button[1]; i++) {
                x = x + TSC_BUTTON_X_RANGE;
            }
            /* Calculate the weighted distance. */
            x = x - (TSC_BUTTON_X_RANGE / 2) * (max[0] - max[1]) / 100; 
        }
    }
    return x;
}

/*
 *  Workq handler
 */
static void 
tm1129_work_handler (struct work_struct *work)
{
    u8 irq_status, status, data[6];
    struct tsc_drv_data *dev;
    static int touch, W, X, Y, Z, X2, Y2;
    int x;

    /* get device */
    dev = container_of(work, struct tsc_drv_data, workq);

    /* Read the Interrupt status register. */
    tm1129_reg_read(dev, INTERRUPT_REQ_STATUS,  &irq_status,  &status);

    /* Handle ABS interrupt */
    if (irq_status & INTERRUPT_REQ_STATUS_ABS_INT_RQ) {

        /* Read six bytes starting from DATA_REG_0. */
        tm1129_reg_read_six(dev, DATA_REG_0,  &data[0],  &status);

        status = data[0] & DATA_REG_0_SENSOR_STATUS;

        if ((status != 0) && (status != TRANSITIONAL_FINGER_COUNT)) {
            touch = status;
        } else {
            Z = 0;
            touch = 0;
        }

        if (touch) {
            /* Only update the parameters if there is actually a
             * touch.  The reason is that we don't want to report
             * bogus data when the finger is lifted.  So when it
             * is lifted, we report the last known good samples.
             */
            W = (data[0] >> 4) & 0xf;
            Z = data[1];
            X = (data[2] & 0x1f) << 8;
            X |= data[3];
            Y = (data[4] & 0x1f) << 8;
            Y |= data[5];

            if (dev->pdata->flip_y) {
                Y = TSC_MAX_ABS_Y - Y;
            }
            if (dev->pdata->flip_x) {
                X = TSC_MAX_ABS_X - X;
            }

        } else {
            /* Logically, we shouldn't be here. Jump out
             * and check remaining interrupt sources. Also
             * set "touch" to zero so that we won't report
             * ghost pen-up/pen-down events.
             */
#ifdef MODDEBUG
            printk(KERN_ERR "%s: Got ABS interrupt with finger value zero. "
                            "(%s:%d)\n", DRIVER,__FILE__, __LINE__);
#endif
            touch = 0;
            goto rel_motion_intr;
        }

        if (touch > 1) {

            /* Read six bytes starting from DATA_REG_6. */
            tm1129_reg_read_six(dev, DATA_REG_6,  &data[0],  &status);

            X2 = (data[2] & 0x1f) << 8;
            X2 |= data[3];
            Y2 = (data[4] & 0x1f) << 8;
            Y2 |= data[5];

            if (dev->pdata->flip_y) {
                Y2 = TSC_MAX_ABS_Y - Y2;
            }
            if (dev->pdata->flip_x) {
                X2 = TSC_MAX_ABS_X - X2;
            }

            input_report_abs( dev->indev, ABS_FINGERS, touch);
            input_report_abs( dev->indev, ABS_BB_X1, X);
            input_report_abs( dev->indev, ABS_BB_Y1, Y);
            input_report_abs( dev->indev, ABS_BB_X2, X2);
            input_report_abs( dev->indev, ABS_BB_Y2, Y2);
            input_report_abs( dev->indev, ABS_TOOL_WIDTH, W);
            input_report_abs( dev->indev, ABS_PRESSURE, Z);
        } else {
            input_report_abs( dev->indev, ABS_FINGERS, touch);
            input_report_abs( dev->indev, ABS_X, X);
            input_report_abs( dev->indev, ABS_Y, Y);
            input_report_abs( dev->indev, ABS_TOOL_WIDTH, W);
            input_report_abs( dev->indev, ABS_PRESSURE, Z);
        }
    }

rel_motion_intr:

    /* Handle REL interrupt. We don't enable it so it shouldn't
     * fire during normal operation except during reset. Read
     * motion related data registers to deassert the interrupt.
     */
    if (irq_status & INTERRUPT_REQ_STATUS_REL_INT_RQ) {
        tm1129_reg_read(dev, REL_HRZ_MOTION,  &data[0],  &status);
        tm1129_reg_read(dev, REL_VRT_MOTION,  &data[1],  &status);
    }

    /* Handle Button interrupt.
     */
    if (irq_status & INTERRUPT_REQ_STATUS_BTN_INT_RQ) {
        tm1129_reg_read_six(dev, BUTTON_DATA,  &data[0],  &status);

        x = x_scroller(dev->cap_btn, &data[1]);

        if (x >=0) {
            input_report_abs( dev->indev, ABS_SCROLL, x);
        }
    }

    /* If the pen was not previously down, generate a pen down event.
     * Do it only for a valid touch and avoid ghost pen-up/pen-down
     * events that result from Synaptics FW issue.
     */
    if ((dev->pen_state == TSC_PEN_UP) && touch) {
        dev->pen_state  = TSC_PEN_DOWN;
        input_report_key( dev->indev, BTN_TOUCH, 1 );
    }
    input_sync( dev->indev);

#ifdef PRINT_EVENTS
    printk( KERN_INFO "%s: (%d, %d, %d)\n", DRIVER, x, y, pressure );
#endif    

    /* Kick the pen up timer. */
    mod_timer( &dev->pen_timer, 
                jiffies + msecs_to_jiffies(dev->pdata->penup_timeout));

    return;   
}

/*
 *  Timer callback
 */
static void 
tm1129_pen_timer ( unsigned long data )
{
    struct tsc_drv_data *dev = (struct tsc_drv_data *) data;

    del_timer_sync ( &dev->pen_timer );
    dev->pen_state = TSC_PEN_UP;
    input_report_key( dev->indev, BTN_TOUCH, 0);
    input_sync( dev->indev);

#ifdef PRINT_EVENTS
    printk( KERN_INFO "%s: penup\n", DRIVER );
#endif    
}

/*
 *  Interrupt handler
 */
static irqreturn_t 
tm1129_irq(int irq, void *dev_id )
{
    struct tsc_drv_data *dev = (struct tsc_drv_data *)dev_id;

    del_timer_sync ( &dev->pen_timer );
    schedule_work  ( &dev->workq );
    return IRQ_HANDLED;
}

/*
 *  Reset controller by writing to reset bit.
 */
static int 
tm1129_controller_reset ( struct tsc_drv_data *dev )
{
    u8 status;
    int timeout;

    /* Reset the touch screen. */
    tm1129_reg_write (dev, DEVICE_COMMAND,
                      DEVICE_COMMAND_RESET, NULL);

    /* Wait until error status register indicates reset 
     * i.e. Look for the 8th bit of the register set.
     */
    timeout = 100;
    while ( -- timeout ) {
        udelay ( 100 );
        tm1129_reg_read(dev, ERROR_STATUS, &status, NULL);
        if( status & (1 << 7))
            return 0;
    }
    return -1;
}

#ifdef MODDEBUG
void
dbg_print(u16 reg, u8 *data, u8 count)
{
    int i;

    printk("Reg: 0x%04x: ", reg);

    /* For read data that's read, starting
     * printing from the 3rd byte.
     */
    for (i = 0; i < count; i++) {
        printk("%02x", data[i+2]);
    }
    printk("   status=%x\n", data[1]);
}

static void 
tm1129_do_query( struct tsc_drv_data *dev )
{
    u8 data[9];
    u8 rmi_maj_ver, rmi_min_ver, manfid, phy_maj_ver;
    u8 phy_min_ver, properties, prod_ver[4];

    tm1129_reg_read_six(dev, RMI_PROTOCOL_VERSION, &data[0], NULL);
    tm1129_reg_read(dev, PROD_INFO_2, &data[6], NULL);
    tm1129_reg_read(dev, PROD_INFO_3, &data[7], NULL);


    rmi_maj_ver = data[0] >> 4;
    rmi_min_ver = data[0] & 0xf;
    manfid      = data[1];
    phy_maj_ver = data[2] >> 4;
    phy_min_ver = data[2] & 0xf;
    properties  = data[3];
    prod_ver[0] = data[4];
    prod_ver[1] = data[5];
    prod_ver[2] = data[0];
    prod_ver[3] = data[1];

    printk("%s: RMI Protocol: %d.%d. Manufacturer ID: %d\n", DRIVER,
           rmi_maj_ver, rmi_min_ver, manfid);

    printk("%s: Physical Interface Version: %d.%d. Product Ver: %d.%d.%d.%d\n",
           DRIVER, phy_maj_ver, phy_min_ver, prod_ver[0], prod_ver[1],
           prod_ver[2], prod_ver[3]);

    printk("%s: Properties: Can Doze: %d\n", DRIVER,
           ((properties & 0x02) == 0x02));

    tm1129_reg_read_six(dev, FUNCTION_VERSION, &data[0], NULL);
    tm1129_reg_read(dev, SENSOR_Y_MAX_HIGH, &data[6], NULL);
    tm1129_reg_read(dev, SENSOR_Y_MAX_LOW, &data[7], NULL);
    tm1129_reg_read(dev, SENSOR_RESOLUTION, &data[8], NULL);

    printk("%s: Function version: %d.%d\n", DRIVER, (data[0] >> 4) & 0xf, data[0] & 0xf);
    printk("%s: Has 2D Scroll: %d\n", DRIVER, (data[3] & SENSOR_PROP_2D_SCROLL) ? 1 : 0);
    printk("%s: Has Scroller: %d\n", DRIVER, (data[3] & SENSOR_PROP_SCROLLER) ? 1 : 0);
    printk("%s: Has Multifing: %d\n", DRIVER, (data[3] & SENSOR_PROP_MULTI_FINGER) ? 1 : 0);
    printk("%s: Has Palm Det: %d\n", DRIVER, (data[3] & SENSOR_PROP_PALM_DET) ? 1 : 0);
    printk("%s: Sensor Max X: %d Max Y: %d\n", DRIVER,
                ((data[4] & 0x1f) << 8) | (data[5] & 0xff),
                ((data[6] & 0x1f) << 8) | (data[7] & 0xff));
    printk("%s: Sensor Resolution: %d\n", DRIVER, data[8]);

    return;
}
#endif

static int 
tm1129_do_resume ( struct tsc_drv_data *dev )
{
    int result = 0;

    /* Restart the controller. */
    result = tm1129_controller_reset ( dev );
    if(result != 0)
    {
        printk(KERN_ERR "%s: Failed to reset controller. (%s:%d)\n",
               DRIVER,__FILE__, __LINE__);
        return result;
    }

    /* Normal operation mode. */
    tm1129_reg_write (dev, DEVICE_CONTROL,
                      (SAMPLE_RATE_80HZ | NORMAL_OPERATION), NULL);

    /* Enable interrupt. */
    tm1129_enable_irq ( dev );

    return 0;
}

static void
tm1129_do_suspend ( struct tsc_drv_data *dev )
{
    tm1129_disable_irq ( dev );  // first
    cancel_work_sync ( &dev->workq );  // cancel work
    del_timer_sync   ( &dev->pen_timer ); // then delete timer

    /* log pen up event */
    if( dev->pen_state == TSC_PEN_DOWN ) {   
        dev->pen_state  = TSC_PEN_UP;
        input_report_key( dev->indev, BTN_TOUCH, 0);
        input_sync( dev->indev);
    }

    /* Deep sleep mode. */
    tm1129_reg_write (dev, DEVICE_CONTROL,
                      (SAMPLE_RATE_80HZ | DEEP_SLEEP), NULL);
}

/*
 *  Setup input device
 */
static int __init 
tm1129_init_input ( struct tsc_drv_data *dev )
{
    int    rc;
    struct input_dev *idev;

    idev = input_allocate_device ();
    if (!idev)
        return -ENOMEM;

    /* setup input device */
    set_bit(EV_KEY,    idev->evbit );
    set_bit(BTN_TOUCH, idev->keybit );

    set_bit(EV_ABS,       idev->evbit );
    set_bit(ABS_X,        idev->absbit );
    set_bit(ABS_Y,        idev->absbit );
    set_bit(ABS_PRESSURE, idev->absbit );
    set_bit(ABS_TOOL_WIDTH, idev->absbit );
    set_bit(ABS_FINGERS, idev->absbit );
    set_bit(ABS_BB_X1, idev->absbit );
    set_bit(ABS_BB_X2, idev->absbit );
    set_bit(ABS_BB_Y1, idev->absbit );
    set_bit(ABS_BB_Y2, idev->absbit );
    set_bit(ABS_SCROLL, idev->absbit );

    input_set_abs_params(idev, ABS_X, 0, TSC_MAX_ABS_X, 0, 0);
    input_set_abs_params(idev, ABS_Y, 0, TSC_MAX_ABS_Y, 0, 0);
    input_set_abs_params(idev, ABS_PRESSURE, 0, TSC_MAX_ABS_Z, 0, 0);
    input_set_abs_params(idev, ABS_TOOL_WIDTH, 0, TSC_MAX_ABS_W, 0, 0);
    input_set_abs_params(idev, ABS_FINGERS, 0, TSC_MAX_ABS_F, 0, 0);
    input_set_abs_params(idev, ABS_BB_X1, 0, TSC_MAX_ABS_X, 0, 0);
    input_set_abs_params(idev, ABS_BB_X2, 0, TSC_MAX_ABS_X, 0, 0);
    input_set_abs_params(idev, ABS_BB_Y1, 0, TSC_MAX_ABS_Y, 0, 0);
    input_set_abs_params(idev, ABS_BB_Y2, 0, TSC_MAX_ABS_Y, 0, 0);
    input_set_abs_params(idev, ABS_SCROLL, 0, TSC_MAX_ABS_X, 0, 0);

    idev->name = DEVICE;
    idev->phys = DEVICE;

    /* Now we are using synaptics part. Do we need to change this? */
    idev->id.bustype  = BUS_HOST;
    idev->id.vendor   = 0x0001;
    idev->id.product  = 0x0001;
    idev->id.version  = 0x0100;
 
    rc = input_register_device ( idev );
    if( rc < 0 ) {
        printk(KERN_ERR "%s: Unable to register input device\n", DRIVER );
        input_free_device ( idev );
        return rc;
    }

    // attach input device to platform device
    dev->indev = idev;
    
    return 0;
}



#ifdef CONFIG_PM

static int 
tm1129_suspend ( struct spi_device *spi, pm_message_t mesg )
{
    struct tsc_drv_data *dev = (struct tsc_drv_data *) spi;

    if(!dev->suspended ) {
        tm1129_do_suspend ( dev );
        dev->suspended = 1;
    }
    
    return 0;
}

static int 
tm1129_resume ( struct spi_device *spi  )
{
    struct tsc_drv_data *dev = (struct tsc_drv_data *) spi;
    
    if( dev->suspended ) {
        tm1129_do_resume ( dev );
        dev->suspended = 0;
    }
    
    return 0;
}

#else

#define tm1129_suspend  NULL
#define tm1129_resume   NULL

#endif /* CONFIG_PM */



static char *
tm1129_get_param(const char *str, s32 *pint)
{
    char  *end;
	while (isspace(*str)) str++;
	*pint = simple_strtol( str, &end, 0);
	return end;
}


static int 
tm1129_transform_show ( struct class_device *cdev, char *buf)
{
    int len = 0;

    len  = sprintf ( buf + len, "%4d ", tformX[0] );
    len += sprintf ( buf + len, "%4d ", tformX[1] );
    len += sprintf ( buf + len, "%4d ", tformX[2] );
    len += sprintf ( buf + len, "%4d ", tformY[0] );
    len += sprintf ( buf + len, "%4d ", tformY[1] );
    len += sprintf ( buf + len, "%4d ", tformY[2] );
    len += sprintf ( buf + len, "%4d\n",tformDiv  );

    return len;
}


static     ssize_t 
tm1129_transform_store ( struct class_device *dev,
                          const char * buf, size_t count  )
{
    int i;
    s32 params[7];

    memset ( params, 0, sizeof(params));
    
    for ( i = 0; i < 7 && *buf; i++ ) {
       buf = tm1129_get_param ( buf, params + i );
    }

    if( params[6] )
    {   // X
        tformX[0] = params[0];
        tformX[1] = params[1];
        tformX[2] = params[2];
        // Y
        tformY[0] = params[3];
        tformY[1] = params[4];
        tformY[2] = params[5];
        // scale
        tformDiv  = params[6];
    }
    return count;
}

static struct class_device_attribute ts_transform =
{
    .attr   = {
        .name  = "transform",
        .mode  =  0644,
        .owner =  THIS_MODULE,
     },
    .show   = tm1129_transform_show,
    .store  = tm1129_transform_store,
};

static struct attribute *tm1129_attributes[] = {
    &ts_transform.attr,
    NULL
};

static int __devexit 
tm1129_remove ( struct spi_device *spi )
{
    struct tsc_drv_data *dev = spi_get_drvdata ( spi );

    PDBG("%s:\n", __FUNCTION__ );

    // remove group
    sysfs_remove_group ( &dev->indev->dev.kobj, &dev->attrs );

    // detach drvdata 
    spi_set_drvdata ( spi, NULL );

    tm1129_do_suspend  ( dev );

    // free irq and gpio
    free_irq  ( spi->irq, dev );
    gpio_free ( irq_to_gpio(spi->irq));

    // Detach and unregister input device
    if( dev->indev ) {
        input_unregister_device ( dev->indev );
        dev->indev = NULL;
    }

    // free context
    if( dev ) 
        kfree ( dev );

    return 0;
}


static int __init 
tm1129_probe(struct spi_device *spi)
{
    int    rc;
    struct tsc_drv_data *dev;
    struct tm1129_platform_data *pdata = spi->dev.platform_data;
    
    PDBG("%s:\n", __FUNCTION__ );

    if( pdata == NULL ) {
        printk ( KERN_ERR "%s: platform data is undefined\n", DRIVER );
        return -ENODEV;
    }

    /* allocate device context */
    dev = kzalloc ( sizeof(struct tsc_drv_data), GFP_KERNEL);
    if( dev == NULL )
        return -ENOMEM;
    dev->spidev = spi;    /* attach spi device */
    dev->pdata  = pdata;  /* and platform specific data */

    /* The max_z and min_z were obtained by plotting raw Z
     * value data points obtained from (badly) tuned sensor.
     * If sensor were tuned properly, we wouldn't need to define
     * separate min/max Z values.
     */
    dev->cap_btn[0].min_z = 10;
    dev->cap_btn[1].min_z = 10;
    dev->cap_btn[2].min_z = 10;
    dev->cap_btn[3].min_z = 10;
    dev->cap_btn[4].min_z = 10;

    dev->cap_btn[0].max_z = 120; 
    dev->cap_btn[1].max_z = 80;
    dev->cap_btn[2].max_z = 55; 
    dev->cap_btn[3].max_z = 60; 
    dev->cap_btn[4].max_z = 70; 

     /* Init spi interface */
    spi->mode = SPI_CPOL | SPI_CPHA; // CPOL=1 base value of the SCLK is one.
                                     // CPHA=1 data read on the rising edge. 
    spi->bits_per_word = 8; 
    spi->max_speed_hz = 2000000;
    rc = spi_setup ( spi );
    if( rc < 0) {
        goto err1;
    }

    /* Create input device */
    rc = tm1129_init_input ( dev );
    if( rc < 0 ) {
        printk(KERN_ERR "%s: Failed to register input device\n", DRIVER );
        goto err1;
    }

    /* set up interrupt handler */
    rc = gpio_request ( irq_to_gpio(spi->irq), "tm1129 irq" );
    if( rc != 0) {
        printk(KERN_ERR "%s: Failed to get GPIO for reset line.\n", DRIVER );
        goto err2;
    }
    rc = request_irq ( spi->irq, tm1129_irq, 
                       IRQF_DISABLED | IRQF_TRIGGER_FALLING,
                       "touchscreen", dev );
    if( rc != 0) {
        printk(KERN_ERR "%s: Failed to get IRQ.\n", DRIVER );
        goto err3;
    }
    tm1129_disable_irq ( dev );

    // setup workq
    INIT_WORK(&dev->workq, tm1129_work_handler );

    // setup timer
    setup_timer ( &dev->pen_timer, tm1129_pen_timer, (unsigned long) dev );

    // attach drv data to spi device
    spi_set_drvdata ( spi, dev );

    /* create attributes */
    dev->attrs.name  = "calibration";
    dev->attrs.attrs =  tm1129_attributes;
    rc = sysfs_create_group ( &dev->indev->dev.kobj,  &dev->attrs );
    if( rc != 0 ) 
        goto err4;

    /* start things up */
    rc = tm1129_do_resume  ( dev );

#ifdef MODDEBUG
    /* query the device info. */
    tm1129_do_query( dev );
#endif

    if( rc < 0 ) 
        goto err5;

    return 0;

err5:
    sysfs_remove_group ( &dev->indev->dev.kobj, &dev->attrs );
err4:
    free_irq ( spi->irq, dev );
err3:
    gpio_free ( irq_to_gpio(spi->irq));
err2:
    input_unregister_device ( dev->indev );
err1:
    kfree ( dev );
    return -ENODEV;
}

static struct spi_driver tm1129_driver = {
    .driver = {
        .name   = DRIVER,
        .bus    = &spi_bus_type,
        .owner  = THIS_MODULE,
    },
    .suspend    = tm1129_suspend,
    .resume     = tm1129_resume,
    .probe      = tm1129_probe,
    .remove     = __devexit_p(tm1129_remove),
};

static int __init  tm1129_init(void)
{
    PDBG("%s:\n", __FUNCTION__ );
    return spi_register_driver ( &tm1129_driver );
}


static void __exit tm1129_exit(void)
{
    PDBG("%s:\n", __FUNCTION__ );
    spi_unregister_driver ( &tm1129_driver );

}

module_init(tm1129_init);
module_exit(tm1129_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

