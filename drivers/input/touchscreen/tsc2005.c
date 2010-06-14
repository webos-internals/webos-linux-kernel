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
#include <linux/spi/tsc2005.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>

#include <asm/arch/gpio.h>

#undef  MODDEBUG
//#define MODDEBUG  1

#ifdef  MODDEBUG
#define PDBG(args...)   printk(args)
#else
#define PDBG(args...)   
#endif

#undef  PRINT_EVENTS
// #define PRINT_EVENTS   1

/*
 * Constants
 */
#define  TSC_NAME       "tsc2005"
#define  DEVICE         TSC_NAME
#define  DRIVER         TSC_NAME
#define  DRIVER_DESC    TSC_NAME" touchscreen driver"

#define  TSC_MAX_ABS    4096  // for 12-bit convertion 

#define  TSC_MAX_ABS_X  TSC_MAX_ABS   
#define  TSC_MAX_ABS_Y  TSC_MAX_ABS  
#define  TSC_MAX_ABS_Z  1024  // for some reason
/*
 * Settings  
 */
#define TSC_CFR0_INIT               \
    (TSC_CFR0_PSM_AUTO | TSC_CFR0_RM_12BIT | TSC_CFR0_CLK_1MHZ | \
     TSC_CFR0_VSTAB_100US | TSC_CFR0_CHRG_84US | TSC_CFR0_SNS_96US)

#define TSC_CFR1_INIT               \
    (TSC_CFR1_TBM_DEFAULT | TSC_CFR1_BDLY_50_SSPS)

#define TSC_CFR2_INIT               \
    (TSC_CFR2_PINTS_DAVONLY | TSC_CFR2_MAVCTL_FLTSZ_3 | \
     TSC_CFR2_MAVCTL_WINSZ_1 | TSC_CFR2_MAVENX | TSC_CFR2_MAVENY | \
     TSC_CFR2_MAVENZ | TSC_CFR2_MAVENAUX | TSC_CFR2_MAVENTMP)

/*
 * Definitions and Prototypes
 */
typedef enum {TSC_PEN_UP, TSC_PEN_DOWN} TSC_PEN_STATE;

typedef struct tsc_drv_data {
    struct spi_device *spidev;
    struct input_dev  *indev;
    struct tsc2005_platform_data *pdata;
    TSC_PEN_STATE      pen_state;
    struct timer_list  pen_timer;
    struct work_struct workq;
    int                suspended;
    DECLARE_BITMAP(gpio_irq_disabled, 1);
    struct attribute_group attrs;
} tsc_drv_data_t;

/* Parameters for transforming X,Y points; defaults leave X,Y unchanged */
static s32 tformX[] = { 4096,     0,  0 };
static s32 tformY[] = {    0,  4096,  0 };
static u32 tformDiv =   4096;     // transform divider

static int tsc2005_cmd      ( struct tsc_drv_data *dev, u32 cmd);
static int tsc2005_reg_read ( struct tsc_drv_data *dev, u32 reg, u32 *data);
static int tsc2005_reg_write( struct tsc_drv_data *dev, u32 reg, u32  data);

/*
 *  Enable IRQ
 */
static void 
tsc2005_enable_irq( struct tsc_drv_data *dev )
{
    if( test_and_clear_bit(0, dev->gpio_irq_disabled))
        enable_irq ( dev->spidev->irq );
}

/*
 *  Disable IRQ
 */
static void 
tsc2005_disable_irq ( struct tsc_drv_data *dev )
{
    if(!test_and_set_bit(0, dev->gpio_irq_disabled))
        disable_irq( dev->spidev->irq  );
}

/*
 *  tsc2005 sync read register routine
 */
static int 
tsc2005_reg_read ( struct tsc_drv_data *dev, u32 reg, u32 *data)
{
    struct spi_message  m;
    struct spi_transfer t[2];

    reg   = TSC_CTL_READ | reg;
    *data = 0;

    memset(t, 0, sizeof(t));
    spi_message_init(&m);

    t[0].bits_per_word = TSC_CTL_SIZE_BITS;
    t[0].tx_buf = &reg;
    t[0].len    = (t[0].bits_per_word + 7) >> 3;
    spi_message_add_tail(&t[0], &m);

    t[1].bits_per_word = TSC_REG_SIZE_BITS;
    t[1].rx_buf = data;
    t[1].len    = ((TSC_REG_SIZE_BITS + 7) >> 3);
    spi_message_add_tail(&t[1], &m);

    return spi_sync ( dev->spidev, &m);
}


/*
 *  tsc2005 sync write register routine
 */
static int 
tsc2005_reg_write ( struct tsc_drv_data *dev, u32 reg, u32 data)
{
    u32 value   = (((TSC_CTL_WRITE | reg) << TSC_REG_SIZE_BITS) | data);
    struct spi_message    m;
    struct spi_transfer    t;

    spi_message_init ( &m );
    memset ( &t, 0, sizeof(t));

    t.bits_per_word = (TSC_CTL_SIZE_BITS + TSC_REG_SIZE_BITS);
    t.tx_buf = &value;
    t.len    = 4;

    spi_message_add_tail ( &t, &m );
    return spi_sync ( dev->spidev, &m );
}

/*
 *  tsc2005 command write routine
 */
static int 
tsc2005_cmd ( struct tsc_drv_data *dev, u32 cmd)
{
    struct spi_message    m;
    struct spi_transfer    t;

    spi_message_init ( &m );
    memset ( &t, 0, sizeof(t));

    t.bits_per_word = TSC_CTL_SIZE_BITS;
    t.len = (t.bits_per_word + 7) >> 3;
    t.tx_buf = &cmd;

    spi_message_add_tail ( &t, &m );
    return spi_sync ( dev->spidev, &m );
}


/*
 *  Workq handler
 */
static void 
tsc2005_work_handler (struct work_struct *work)
{
    u32 dev_x, dev_y, dev_z1, dev_z2;
    s32 x, y;
    struct tsc_drv_data *dev;
    int pressure = 0;

    /* get device */
    dev = container_of(work, struct tsc_drv_data, workq);

    /* Read in the values to quiesce the level interrupt line. */
    (void) tsc2005_reg_read(dev, TSC_REG_XDATA,  &dev_x  );
    (void) tsc2005_reg_read(dev, TSC_REG_YDATA,  &dev_y  );
    (void) tsc2005_reg_read(dev, TSC_REG_Z1DATA, &dev_z1 );
    (void) tsc2005_reg_read(dev, TSC_REG_Z2DATA, &dev_z2 );

    /* Calculate the pressure. */
    if( dev_z1 != 0) {
        pressure = ((dev->pdata->xresistence * dev_x * dev_z2) / (TSC_MAX_ABS * dev_z1)) -
                   ((dev->pdata->xresistence * dev_x)          / (TSC_MAX_ABS));
        if( pressure < 0 )
            pressure = 0;
    }

    /* Y-Axis is inverted */
    dev_y = TSC_MAX_ABS - dev_y - 1;

    /* Apply x,y transform parameters */
    x = ((s32)dev_x * tformX[0] + (s32)dev_y * tformX[1] + tformX[2]) / tformDiv;
    y = ((s32)dev_x * tformY[0] + (s32)dev_y * tformY[1] + tformY[2]) / tformDiv;

    /* force the range */
    if( x < 0 ) x = 0;
    if( x >= TSC_MAX_ABS_X ) x = TSC_MAX_ABS_X-1;
    
    if( y < 0 ) y = 0;
    if( x >= TSC_MAX_ABS_Y ) x = TSC_MAX_ABS_Y-1;

    /* Report event. */
    input_report_abs( dev->indev, ABS_X, x );
    input_report_abs( dev->indev, ABS_Y, y );
    input_report_abs( dev->indev, ABS_PRESSURE, pressure);

    /* If the pen was not previously down generate a pen down event. */
    if( dev->pen_state == TSC_PEN_UP) {
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
tsc2005_pen_timer ( unsigned long data )
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
tsc2005_irq(int irq, void *dev_id )
{
    struct tsc_drv_data *dev = (struct tsc_drv_data *)dev_id;

    del_timer_sync ( &dev->pen_timer );
    schedule_work  ( &dev->workq );
    return IRQ_HANDLED;
}

/*
 *  Pulse GPIO to reset controller.
 */
static int 
tsc2005_controller_reset ( struct tsc_drv_data *dev )
{
    u32 status;
    int timeout;
    int reset_gpio  = dev->pdata->reset_gpio;
    int reset_level = dev->pdata->reset_level;

    gpio_set_value ( reset_gpio, !reset_level ); 
    udelay(15);
    gpio_set_value ( reset_gpio,  reset_level ); 
    udelay(15);
    gpio_set_value ( reset_gpio, !reset_level );

    /* read status wait until it is reset */
    timeout = 100;
    while ( -- timeout ) {
        udelay ( 100 );
        tsc2005_reg_read(dev, TSC_REG_STATUS, &status );
        if( status & (1 << 7))
            return 0;
    }
    return -1;
}

static int 
tsc2005_do_resume ( struct tsc_drv_data *dev )
{
    int result = 0;

    /* Restart the controller. */
    result = tsc2005_controller_reset ( dev );
    if(result != 0)
    {
        printk(KERN_ERR "%s: Failed to reset controller. (%s:%d)\n",
               DRIVER,__FILE__, __LINE__);
        return result;
    }

    /* Configure the controller. */
    result  = tsc2005_reg_write(dev, TSC_REG_CFR0, TSC_CFR0_INIT);
    result |= tsc2005_reg_write(dev, TSC_REG_CFR1, TSC_CFR1_INIT);
    result |= tsc2005_reg_write(dev, TSC_REG_CFR2, TSC_CFR2_INIT);
    if(result != 0) {
        printk(KERN_ERR "%s: Failed to write configuration. (%s:%d)\n",
               DRIVER,__FILE__, __LINE__);
        return result;
    }

/*
    {
        u32 cfr0, cfr1, cfr2;
        tsc2005_reg_read (dev, TSC_REG_CFR0, &cfr0 );
        tsc2005_reg_read (dev, TSC_REG_CFR1, &cfr1 );
        tsc2005_reg_read (dev, TSC_REG_CFR2, &cfr2 );
        printk ( "CFR0=0x%08x[0x%08x]\n", cfr0, TSC_CFR0_INIT);
        printk ( "CFR1=0x%08x[0x%08x]\n", cfr1, TSC_CFR1_INIT);
        printk ( "CFR2=0x%08x[0x%08x]\n", cfr2, TSC_CFR2_INIT);
    }
*/    

    tsc2005_enable_irq ( dev );

    /* Go. */
    result = tsc2005_cmd ( dev, TSC_CMD_XYZSCAN );
    if( result != 0) {
        printk(KERN_ERR "%s: Failed to send command. (%s:%d)\n",
               DRIVER,__FILE__, __LINE__);
        return result;
    }

    return 0;
}

static void
tsc2005_do_suspend ( struct tsc_drv_data *dev )
{
    int rc;

    tsc2005_disable_irq ( dev );  // first
    cancel_work_sync ( &dev->workq );  // cancel work
    del_timer_sync   ( &dev->pen_timer ); // then delete timer
    
    /* log pen up event */
    if( dev->pen_state == TSC_PEN_DOWN ) {   
        dev->pen_state  = TSC_PEN_UP;
        input_report_key( dev->indev, BTN_TOUCH, 0);
        input_sync( dev->indev);
    }
    
    /* Issue command to stop conversions and power down A/D. */
    rc = tsc2005_cmd ( dev, TSC_CMD_STOP_AND_POWER_DOWN );
    if( rc != 0) {
        printk( KERN_ERR "%s: Failed to send command. (%s:%d)\n",
                DRIVER, __FILE__, __LINE__);
    }

    /* Place the controller into reset. */
    gpio_set_value ( dev->pdata->reset_gpio, dev->pdata->reset_level );
}

/*
 *  Setup input device
 */
static int __init 
tsc2005_init_input ( struct tsc_drv_data *dev )
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

    input_set_abs_params(idev, ABS_X,        0, TSC_MAX_ABS_X, 0, 0);
    input_set_abs_params(idev, ABS_Y,        0, TSC_MAX_ABS_Y, 0, 0);
    input_set_abs_params(idev, ABS_PRESSURE, 0, TSC_MAX_ABS_Z, 0, 0);

    idev->name = DEVICE;
    idev->phys = DEVICE;

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
tsc2005_suspend ( struct spi_device *spi, pm_message_t mesg )
{
    struct tsc_drv_data *dev = spi_get_drvdata(spi);

    if(!dev->suspended ) {
        tsc2005_do_suspend ( dev );
        dev->suspended = 1;
    }
    
    return 0;
}

static int 
tsc2005_resume ( struct spi_device *spi  )
{
    struct tsc_drv_data *dev = spi_get_drvdata(spi);
    
    if( dev->suspended ) {
        tsc2005_do_resume ( dev );
        dev->suspended = 0;
    }
    
    return 0;
}

#else

#define tsc2005_suspend  NULL
#define tsc2005_resume   NULL

#endif /* CONFIG_PM */



static char *
ts2005_get_param(const char *str, s32 *pint)
{
    char  *end;
	while (isspace(*str)) str++;
	*pint = simple_strtol( str, &end, 0);
	return end;
}


static int 
tsc2005_transform_show ( struct class_device *cdev, char *buf)
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
tsc2005_transform_store ( struct class_device *dev,
                          const char * buf, size_t count  )
{
    int i;
    s32 params[7];

    memset ( params, 0, sizeof(params));
    
    for ( i = 0; i < 7 && *buf; i++ ) {
       buf = ts2005_get_param ( buf, params + i );
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
    .show   = tsc2005_transform_show,
    .store  = tsc2005_transform_store,
};

static struct attribute *ts2005_attributes[] = {
    &ts_transform.attr,
    NULL
};

static int __devexit 
tsc2005_remove ( struct spi_device *spi )
{
    struct tsc_drv_data *dev = spi_get_drvdata ( spi );
    struct tsc2005_platform_data *pdata = spi->dev.platform_data;

    PDBG("%s:\n", __FUNCTION__ );

    // remove group
    sysfs_remove_group ( &dev->indev->dev.kobj, &dev->attrs );

    // detach drvdata 
    spi_set_drvdata ( spi, NULL );

    tsc2005_do_suspend  ( dev );

    // free irq and gpio
    free_irq  ( spi->irq, dev );
    gpio_free ( irq_to_gpio(spi->irq));

    // Detach and unregister input device
    if( dev->indev ) {
        input_unregister_device ( dev->indev );
        dev->indev = NULL;
    }

    // free reset gpio
    gpio_free ( pdata->reset_gpio );

    // free context
    if( dev ) 
        kfree ( dev );

    return 0;
}


static int __init 
tsc2005_probe(struct spi_device *spi)
{
    int    rc;
    struct tsc_drv_data *dev;
    struct tsc2005_platform_data *pdata = spi->dev.platform_data;
    
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

    /* Set up reset line. */
    rc = gpio_request ( pdata->reset_gpio, "tsc reset" );
    if( rc != 0) {
        printk(KERN_ERR "%s: Failed to get GPIO for reset line.\n", DRIVER );
        goto err0;
    }
    /* and assert reset */
    gpio_direction_output ( pdata->reset_gpio, pdata->reset_level );

     /* Init spi interface */
    spi->mode = 0 | 0; // PHA:odd, POL:high, CS_POL:low
    spi->bits_per_word = 8; 
    rc = spi_setup ( spi );
    if( rc < 0) {
        goto err1;
    }

    /* Create input device */
    rc = tsc2005_init_input ( dev );
    if( rc < 0 ) {
        printk(KERN_ERR "%s: Failed to register input device\n", DRIVER );
        goto err1;
    }

    /* set up interrupt handler */
    rc = gpio_request ( irq_to_gpio(spi->irq), "tsc2005 irq" );
    if( rc != 0) {
        printk(KERN_ERR "%s: Failed to get GPIO for reset line.\n", DRIVER );
        goto err2;
    }
    rc = request_irq ( spi->irq, tsc2005_irq, 
                       IRQF_DISABLED | IRQF_TRIGGER_FALLING,
                       "touchscreen", dev );
    if( rc != 0) {
        printk(KERN_ERR "%s: Failed to get IRQ.\n", DRIVER );
        goto err3;
    }
    tsc2005_disable_irq ( dev );

    // setup workq
    INIT_WORK(&dev->workq, tsc2005_work_handler );

    // setup timer
    setup_timer ( &dev->pen_timer, tsc2005_pen_timer, (unsigned long) dev );

    // attach drv data to spi device
    spi_set_drvdata ( spi, dev );

    /* create attributes */
    dev->attrs.name  = "calibration";
    dev->attrs.attrs =  ts2005_attributes;
    rc = sysfs_create_group ( &dev->indev->dev.kobj,  &dev->attrs );
    if( rc != 0 ) 
        goto err4;

    /* start things it up */
    rc = tsc2005_do_resume  ( dev );
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
    gpio_free ( pdata->reset_gpio );
err0:
    kfree ( dev );
    return -ENODEV;
}

static struct spi_driver tsc2005_driver = {
    .driver = {
        .name   = DRIVER,
        .bus    = &spi_bus_type,
        .owner  = THIS_MODULE,
    },
    .suspend    = tsc2005_suspend,
    .resume     = tsc2005_resume,
    .probe      = tsc2005_probe,
    .remove     = __devexit_p(tsc2005_remove),
};

static int __init  tsc2005_init(void)
{
    PDBG("%s:\n", __FUNCTION__ );
    return spi_register_driver ( &tsc2005_driver );
}


static void __exit tsc2005_exit(void)
{
    PDBG("%s:\n", __FUNCTION__ );
    spi_unregister_driver ( &tsc2005_driver );

}

module_init(tsc2005_init);
module_exit(tsc2005_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

