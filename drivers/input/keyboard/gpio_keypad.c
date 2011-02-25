/*
 * linux/drivers/input/keyboard/gpio_keypad.c
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keypad.h>

#include <asm/hardware.h>
#include <asm/arch/io.h>
#include <asm/arch/gpio.h>
#include <asm/arch/board.h>

#undef  MODDEBUG
//#define MODDEBUG  1

#undef  REPORT_EVENT
//#define REPORT_EVENT  1

#ifdef  MODDEBUG
#define PDBG(args...)   printk(args)
#else
#define PDBG(args...)   
#endif

#define DEVICE       "gpio_keypad"
#define DRIVER       "gpio_keypad"
#define	DRIVER_DESC	 "Gpio keypad driver"

/* Globals */
spinlock_t kp_lock = SPIN_LOCK_UNLOCKED;

#define KP_MAX_INPUTS 8

typedef struct gpio_kp_dev 
{
    int    scanning;
    int    suspended;
    struct input_dev        *idev;
    struct platform_device  *pdev;
    struct gpio_kp_config   *pcfg;
    struct timer_list       kp_timer;
    struct tasklet_struct   kp_tasklet;
    u16    mx_state [KP_MAX_INPUTS]; // matrix state
    u16    last_scan[KP_MAX_INPUTS]; // last scan results
} gpio_kp_dev_t;

/*************************************************************************
 *
 *  Local funtion prototypes 
 *
 *************************************************************************/

static irqreturn_t kp_interrupt(int irq, void *dev_id );

/*************************************************************************
 *
 *  The following code "might be" somewhat arch specific. At this point
 *  it is generic.
 *
 *************************************************************************/

/*
 *  Set output of single gpio line
 */
#define kp_arch_set_output gpio_set_value

/*
 *  Set all outputs
 */
static inline void
kp_arch_set_outputs ( int *gpios, int cnt, int value )
{
    int i;
    for ( i = 0; i < cnt; i++ ) 
        gpio_set_value ( gpios[i], value );
}

/*
 *  Read all inputs
 */
static inline int
kp_arch_read_inputs ( int *gpios, int cnt )
{
    int i, inputs = 0;
    for ( i = 0; i < cnt; i++ )  {
        if( gpio_get_value ( gpios[i] ) == 0)
            inputs |= (1 << i);
    }
    return inputs;
}

/*
 *  Allocate gpio and setup irq
 */
static int 
kp_arch_init_gpio ( int gpio, int input, void * dev_id )
{
    int rc;
    
    PDBG("%s: %3d (%s) \n", __FUNCTION__, gpio, input ? "input" : "output" );

	if( gpio_request ( gpio, "keypad gpio" ) < 0) 
	    return -1;

    if(!input)
    {   // set output to high
        gpio_direction_output ( gpio, 1 );
    }
    else    
	{   // request IRQ
        gpio_direction_input  ( gpio );
        rc = request_irq ( gpio_to_irq(gpio), kp_interrupt, 
                           IRQF_TRIGGER_FALLING, DRIVER, dev_id );
        if( rc < 0 ) {
            gpio_free ( gpio );
            return rc;
        }
        disable_irq ( gpio_to_irq(gpio));
   	}
	return 0;
}

/*
 *  Free given gpio and release irq
 */
static void
kp_arch_free_gpio ( int gpio, int input, void * dev_id )
{
    PDBG("%s: %3d (%s) \n", __FUNCTION__, gpio, input ? "input" : "output" );

    if( input ) 
        free_irq (  gpio_to_irq(gpio), dev_id );
    gpio_free ( gpio );
}

/*
 *  Disable interrupts 
 */
static inline void
kp_arch_irq_disable ( int *gpios, int cnt )
{
    int i;
    for ( i = 0; i < cnt; i++ ) 
        disable_irq ( gpio_to_irq(gpios[i]));
}

/*
 *  Enable interrupts 
 */
static inline void
kp_arch_irq_enable  ( int *gpios, int cnt )
{
    int i;
    for ( i = 0; i < cnt; i++ ) 
        enable_irq ( gpio_to_irq(gpios[i]));
}


/*
 *  Scan timer related
 *
 *  In this implementation it is kernel timer, but it could be 
 *  OS timer if faster scan is needed, but it can always revert to
 *  kernel timer if OS timer is unavalable.
 *  
 */
static void 
kp_arch_scantimer_callback ( unsigned long data )
{
    struct gpio_kp_dev *kpdev = (struct gpio_kp_dev *) data;
    del_timer ( &kpdev->kp_timer );
	tasklet_schedule ( &kpdev->kp_tasklet );
}

static inline void
kp_arch_scantimer ( struct gpio_kp_dev *kpdev, int set )
{
    if( set )
        mod_timer ( &kpdev->kp_timer, 
                    jiffies + (kpdev->pcfg->debounce * HZ)/1000);
    else
        del_timer ( &kpdev->kp_timer );
}

static void
kp_arch_scantimer_setup ( struct gpio_kp_dev *kpdev )
{
    setup_timer( &kpdev->kp_timer, 
                 kp_arch_scantimer_callback, 
                 (unsigned long)kpdev);
}

static void
kp_arch_scantimer_release ( struct gpio_kp_dev *kpdev )
{
    del_timer_sync ( &kpdev->kp_timer );
}


/*************************************************************************
 *
 *  Local helpers
 *
 *************************************************************************/

/*
 *  Check is any key is down
 */
static int
kp_any_key_down ( int n, u16 *state )
{
    int i;
    u16 down = 0;
    for ( i = 0; i < n; i++ )
        down |= state[i];
    return (int) down;
}

/*
 *  Enter scan mode. It is safe to call it multiple times
 */
static void 
kp_enter_scan_mode ( struct gpio_kp_dev *kpdev )
{
    unsigned long flags;

    spin_lock_irqsave ( &kp_lock, flags );
    if(!kpdev->scanning ) 
    {   // set all outputs high
        kp_arch_set_outputs ( kpdev->pcfg->cols, kpdev->pcfg->col_num, 1 );
        // disable interrupts
        kp_arch_irq_disable ( kpdev->pcfg->rows, kpdev->pcfg->row_num );
        // we are in scan mode
        kpdev->scanning = 1;
    }
    spin_unlock_irqrestore ( &kp_lock, flags );
}

/*
 *  Enter irq mode
 */
static void 
kp_enter_irq_mode ( struct gpio_kp_dev *kpdev )
{
    unsigned long flags;

    spin_lock_irqsave ( &kp_lock, flags );
    if( kpdev->scanning ) 
    {   // enable interrupts
        kp_arch_irq_enable  ( kpdev->pcfg->rows, kpdev->pcfg->row_num );
        // set all outputs low which should trigger an interrupt 
        // if any key is pressed.
        kp_arch_set_outputs ( kpdev->pcfg->cols, kpdev->pcfg->col_num, 0 );
        kpdev->scanning = 0;
    }
    spin_unlock_irqrestore ( &kp_lock, flags );
}
 

/*
 *  Must be called with all row interrupts disabled
 */
static void 
kp_scan_keypad ( struct gpio_kp_dev *kpdev, u16 *state )
{
    int i;
    struct gpio_kp_config *pcfg = kpdev->pcfg;

    // scan
    for( i = 0; i < pcfg->col_num; i++ ) 
    {
        // set one col low
        kp_arch_set_output ( pcfg->cols[i], 0 );

        // let it settle
        udelay ( pcfg->gpio_delay );

        // read inputs
        state[i] = kp_arch_read_inputs ( pcfg->rows, pcfg->row_num );

        kp_arch_set_output ( pcfg->cols[i], 1 );
    }
    udelay ( pcfg->gpio_delay );
}


/*
 *  Compare and copy new to last
 *  Return non zero if arrays are differ 
 */
static u16
kp_cmp_and_copy ( int n, u16 *prev_scan, u16 *new_scan )
{
    int i;
    u16 res = 0;

    for ( i = 0; i < n; i++ ) {
        res |= *prev_scan ^ *new_scan;
       *prev_scan++ = *new_scan++;
    }
    return res;
}

/*
 *  Route key event to whatever destination required
 */
static void
kp_report_key_event ( struct gpio_kp_dev *kpdev, 
                      int scan_code, int key_code, int key_down)
{
#ifdef REPORT_EVENT
    printk( KERN_NOTICE "key_event: sc=%-3d kc=%-3d %s\n",
            scan_code, key_code, key_down ? "pressed":"released");
#endif 

    if( key_code )
	    input_report_key ( kpdev->idev, key_code, key_down );
}

/*
 *  Release all keys
 */
static void
kp_release_all_keys ( struct gpio_kp_dev *kpdev, u16 *state )
{
    int   i, j;
    struct gpio_kp_config *pcfg = kpdev->pcfg;

    for ( i = 0; i < pcfg->col_num; i++ )
    {
        u16 changed = state[i];
        j = 0;
        while ( changed ) 
        {
            if( changed & 1 )
            {
                int scan_code = j * pcfg->col_num + i;
                int key_code  = pcfg->keymap[scan_code];
                int key_down  = (int) (state[i] & (1 << j));
                kp_report_key_event ( kpdev, scan_code, key_code, key_down );
            }
            changed >>= 1; j++;
        }
    }
}


/*
 * 
 */
static void
kp_generate_keys ( struct gpio_kp_dev *kpdev, u16 *prev_scan, u16 *new_scan )
{
    int i, j;
    struct gpio_kp_config *pcfg = kpdev->pcfg;

    // filter out ghost keys
    for ( i = 0; i < pcfg->col_num; i++ )
    {
        if(!(new_scan[i] & (new_scan[i]-1)))
            continue;
        // two or more bits are on the same row 
        // lets see if we can make "square"
        for ( j = 0; j < pcfg->col_num; j++ ) {
            if((i != j) && new_scan[i] & new_scan[j] )
                return; // yeap ghost key
        }
    }

    // generate keys
    for ( i = 0; i < pcfg->col_num; i++ )
    {
        u16 changed = prev_scan[i] ^ new_scan[i];
        j = 0;
        while ( changed ) 
        {
            if( changed & 1 )
            {
                int scan_code = j * pcfg->col_num + i;
                int key_code  = pcfg->keymap[scan_code];
                int key_down  = (int) (new_scan[i] & (1 << j));
                kp_report_key_event ( kpdev, scan_code, key_code, key_down );
            }
            changed >>= 1; j++;
        }
        prev_scan[i] = new_scan[i];
    }
    return;
}

/*
 *  return non-zero if another scan is needed
 */
static int
kp_handle_scan_result ( struct gpio_kp_dev *kpdev, u16 *new_scan )
{
    int res;

    res  = kp_cmp_and_copy ( kpdev->pcfg->col_num, kpdev->last_scan, new_scan );
    if( res ) 
        return 1; // need new scan

    kp_generate_keys    ( kpdev, kpdev->mx_state, kpdev->last_scan );
    if( kp_any_key_down ( kpdev->pcfg->col_num, kpdev->mx_state ))
        return 1;

    return 0;
}

/*
 *  Main keypad scan handler
 */
static void 
kp_scan_handler ( struct gpio_kp_dev *kpdev )
{
    int need_scan;
    u16 new_scan [KP_MAX_INPUTS];

    kp_scan_keypad     ( kpdev, new_scan );
    need_scan = kp_handle_scan_result ( kpdev, new_scan );
    if( need_scan )
        kp_arch_scantimer ( kpdev, 1 ); // start scan timer
    else
        kp_enter_irq_mode ( kpdev );
}

/*
 *  Tasklet callback
 */
static void 
kp_tasklet_callback ( unsigned long data )
{
    kp_scan_handler ((struct gpio_kp_dev *) data );
}

/*
 *  Interrupt Handler
 */
static irqreturn_t 
kp_interrupt(int irq, void *dev)
{
    struct gpio_kp_dev *kpdev = (struct gpio_kp_dev *) dev;
    kp_enter_scan_mode ( kpdev );
    tasklet_schedule   (&kpdev->kp_tasklet);
    
	return IRQ_HANDLED;
}

/*************************************************************************
 *
 *  Driver initilization
 *
 *************************************************************************/

static inline void *
kp_check_cfg ( struct platform_device  *dev )
{
    struct gpio_kp_config *pcfg;

    pcfg = dev->dev.platform_data;
    if( pcfg == NULL       || pcfg->keymap == NULL ||
        pcfg->col_num == 0 || pcfg->cols   == NULL || 
        pcfg->row_num == 0 || pcfg->rows   == NULL )
        return NULL;

    return pcfg;        
}

/*
 *  Initializes all required gpios and irqs
 */
static int 
kp_init_gpios ( struct gpio_kp_dev *kpdev )
{
    int rc, c = 0, r = 0; 

    // outputs    
	for ( c = 0; c < kpdev->pcfg->col_num; c++) {
        rc = kp_arch_init_gpio ( kpdev->pcfg->cols[c], 0, kpdev );	
        if( rc < 0 ) 
            goto err0;
    }
    // inputs
 	for ( r = 0; r < kpdev->pcfg->row_num; r++) {
        rc = kp_arch_init_gpio ( kpdev->pcfg->rows[r], 1, kpdev );
        if( rc < 0 ) 
            goto err1;
    }

    return  0;

err1:
    // unwind rows
    while (--r > 0)
        kp_arch_free_gpio ( kpdev->pcfg->rows[r], 1, kpdev );
err0:
    // unwind columns
    while (--c > 0)
        kp_arch_free_gpio ( kpdev->pcfg->cols[c], 0, kpdev );
    
    return -1;
}

/*
 *  Releases all gpios and irqs
 */
static void 
kp_free_gpios ( struct gpio_kp_dev *kpdev )
{
    int  i;

	for ( i = 0; i < kpdev->pcfg->col_num; i++)
        kp_arch_free_gpio ( kpdev->pcfg->cols[i], 0, kpdev );	

 	for ( i = 0; i < kpdev->pcfg->row_num; i++)
        kp_arch_free_gpio ( kpdev->pcfg->rows[i], 1, kpdev );

    return;        
}

/*
 *  Setup input device
 */
static int __init 
kp_init_input ( struct gpio_kp_dev *kpdev )
{
    int i,rc,keym_len, *keym;
    struct  input_dev  *idev;

	idev = input_allocate_device ();
	if (!idev)
		return -ENOMEM;

	/* setup input device */
	set_bit(EV_KEY, idev->evbit );
	set_bit(EV_REP, idev->evbit );

	keym     = kpdev->pcfg->keymap;
	keym_len = kpdev->pcfg->row_num * kpdev->pcfg->col_num;
	for (i = 0; i < keym_len ; i++)
		set_bit ( keym[i] & KEY_MAX, idev->keybit );

  	idev->name = DEVICE;
    idev->phys = DEVICE"/input0";

	idev->id.bustype  = BUS_HOST;
  	idev->id.vendor   = 0x0001;
  	idev->id.product  = 0x0001;
  	idev->id.version  = 0x0100;

    // Use input device default autorepeat settings
    idev->rep[REP_DELAY]  = 0;
    idev->rep[REP_PERIOD] = 0;

	idev->keycode     = keym;
	idev->keycodemax  = keym_len;
	idev->keycodesize = sizeof(int);

	rc = input_register_device ( idev );
	if( rc < 0 ) {
		printk(KERN_ERR "%s: Unable to register input device\n", DRIVER );
		input_free_device ( idev );
		return rc;
	}

    // 
	idev->rep[REP_DELAY]  = kpdev->pcfg->rep_delay;
	idev->rep[REP_PERIOD] = kpdev->pcfg->rep_period;

    // attach input device to platform device
    kpdev->idev = idev;
    
    return 0;
}


static int __devexit
kp_remove ( struct platform_device *dev )
{
    struct gpio_kp_dev *kpdev;
    
    PDBG("%s:\n", __FUNCTION__ );

    kpdev = platform_get_drvdata ( dev );
    
    // shut things down
	tasklet_disable ( &kpdev->kp_tasklet );
	kp_arch_scantimer_release ( kpdev );
    kp_enter_scan_mode ( kpdev );
    tasklet_kill    ( &kpdev->kp_tasklet );

    // free input device
    if( kpdev->idev ) {
        input_unregister_device ( kpdev->idev );
    }

    // detach 
    platform_set_drvdata ( dev, NULL );

    // free gpios
    kp_free_gpios ( kpdev );

    // free device context
    kfree ( kpdev );

    return 0;    
}

static int __init 
kp_probe ( struct platform_device  *dev )
{
    int rc;
    struct gpio_kp_dev *kpdev = NULL;

    PDBG("%s:\n", __FUNCTION__ );

    /* allocate kpdevice context  */
    kpdev = kzalloc(sizeof(struct gpio_kp_dev), GFP_KERNEL);
    if( kpdev == NULL )
        goto err;
    kpdev->pdev = dev;  /* attach platform device */

    /* check keypad config params  and attach it to kpdev context */
    kpdev->pcfg = kp_check_cfg ( dev );
    if( kpdev->pcfg == NULL ) {
        PDBG( KERN_ERR "%s: Invalid keypad config\n", DRIVER );
        goto err;
    }

    /* configure gpios and setup irq handlers */
    rc = kp_init_gpios ( kpdev );
    if( rc < 0 ) {
        printk( KERN_ERR "%s: Failed (%d) to setup gpios\n", DRIVER, rc );
        goto err;
    }

    /* setup input driver */
    rc = kp_init_input ( kpdev );
    if( rc ) {
        printk( KERN_ERR "%s: Failed (%d) to init input device\n", DRIVER, rc );
        goto err0;
    }

    // attach kpdev context to it to device
    platform_set_drvdata ( dev, kpdev );

    /* setup scan timer */
    kp_arch_scantimer_setup ( kpdev );

    /* setup and enable tasklet */
    tasklet_init( &kpdev->kp_tasklet, kp_tasklet_callback, (unsigned long) kpdev );

    /* at this moment we are in "scanning" mode  
     * (input interrupts are disabled and output gpios pulled up)
     * mark our state as such.  
     */
    kpdev->scanning = 1;

    /* start things up */
    kp_enter_irq_mode  ( kpdev );

    printk ( KERN_INFO "%s: initialized\n", DRIVER_DESC );

	return 0;

err0:
    kp_free_gpios ( kpdev );
err:
    if( kpdev ) 
        kfree ( kpdev );
    return -ENODEV;
}

#ifdef CONFIG_PM
static int 
kp_suspend( struct platform_device *dev, pm_message_t state )
{
    struct gpio_kp_dev *kpdev;
    
    PDBG("%s:\n", __FUNCTION__ );

    kpdev = platform_get_drvdata ( dev );

    if( kpdev->suspended ) 
        return 0;
    
    // suspend
    tasklet_disable     ( &kpdev->kp_tasklet );  // disable tasklet
    kp_arch_scantimer   ( kpdev, 0 );            // stop  scan timer
    kp_enter_scan_mode  ( kpdev );               // enter scan mode 
    tasklet_kill        ( &kpdev->kp_tasklet );  // kill tasklet

    kp_release_all_keys ( kpdev, kpdev->mx_state );
    //  kp_prepare_wakeup   ( kpdev );

    kpdev->suspended = 1;
    
	return 0;
}

static int 
kp_resume ( struct platform_device *dev )
{
    struct gpio_kp_dev *kpdev;
    
    PDBG("%s:\n", __FUNCTION__ );

    kpdev = platform_get_drvdata ( dev );
    if(!kpdev->suspended ) 
        return 0; // already resumed

    memset ( kpdev->mx_state,  0, sizeof(kpdev->mx_state));
    memset ( kpdev->last_scan, 0, sizeof(kpdev->last_scan));
    kp_enter_irq_mode ( kpdev );
    kpdev->suspended = 0;

	return 0;
}
#else
#define kp_suspend  NULL
#define kp_resume   NULL
#endif  /* CONFIG_PM */

static struct platform_driver kp_driver = {
	.driver		= {
		.name	= DRIVER,
	},
	.probe		= kp_probe,
	.remove		= __devexit_p(kp_remove),
	.suspend	= kp_suspend,
	.resume		= kp_resume,
};


static int __devinit gpio_kp_init(void)
{
    PDBG("%s:\n", __FUNCTION__ );
	return platform_driver_register ( &kp_driver );
}

static void __exit   gpio_kp_exit(void)
{
    PDBG("%s:\n", __FUNCTION__ );
    platform_driver_unregister ( &kp_driver ); 
}

module_init(gpio_kp_init);
module_exit(gpio_kp_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");


