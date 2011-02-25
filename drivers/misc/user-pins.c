/*
 * drivers/user-pins.c
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
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/ctype.h>
#include <linux/sysfs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/user-pins.h>

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/board.h>

#undef  MODDEBUG
//#define MODDEBUG  1

#ifdef  MODDEBUG
#define PDBG(args...)   printk(args)
#else
#define PDBG(args...)   
#endif

/*
 *  Register user space hardware subsystem
 */
static decl_subsys(user_hw, NULL, NULL);

static int __init  user_hw_init(void)
{
	return subsystem_register ( &user_hw_subsys );
}

arch_initcall(user_hw_init);


/*
 *  Register gpio pins subsystem
 */
struct pin_attribute {
	struct attribute attr;
	ssize_t (*show) ( struct pin_attribute *attr, char *buf);
	ssize_t (*store)( struct pin_attribute *attr, const char *buf, size_t count );
};

#define to_pin_attr(_attr) container_of(_attr, struct pin_attribute, attr)

static ssize_t
pin_attr_show ( struct kobject * kobj, struct attribute *attr, char * buf)
{
	struct pin_attribute * pin_attr = to_pin_attr(attr);
	if( pin_attr->show)
		return pin_attr->show( pin_attr, buf );
	else
		return -EIO; 
	return 0;
}

static ssize_t
pin_attr_store( struct kobject * kobj, struct attribute * attr, 
                const char * buf, size_t count)
{
	struct pin_attribute * pin_attr = to_pin_attr(attr);
	if( pin_attr->store)
		return pin_attr->store ( pin_attr, buf, count );
	else
		return -EIO;
}

static struct sysfs_ops pin_sysfs_ops = {
	.show	= pin_attr_show,
	.store	= pin_attr_store,
};

static struct kobj_type ktype_pin = {
	.release	= NULL,
	.sysfs_ops	= &pin_sysfs_ops,
};

static decl_subsys ( pins, NULL, NULL );

struct gpio_pin {
	int     gpio;
	int     options;
	int     direction;
	int     act_level;
	int     def_level;
	irqreturn_t (*irq_handler)(int irq, void *data);
	int     irq_config;
	const char * name;
	int     irq_handle_mode;
	int     irqs_during_suspend;
	struct  pin_attribute attr_gpio;
	struct  pin_attribute attr_level;
	struct  pin_attribute attr_active;
	struct  pin_attribute attr_direction;
	struct  pin_attribute attr_irq_handle_mode;
	struct  attribute *attr_ptr_arr[6];
};

struct gpio_pin_set_item {
	struct attribute_group  attr_grp;
	struct gpio_pin         pin;
};

struct gpio_pin_set {
	const  char *set_name;
	struct kobject   kobj;
	int          num_pins;
	struct gpio_pin_set_item pins[];
};

struct gpio_pin_dev_ctxt {
	spinlock_t suspend_lock;
	int    num_sets;
	struct gpio_pin_set *sets[];	
};

/*
 *  Show irq handle mode
 *
 *  If AUTO, irq will be handled by irq_handler 
 */
static int 
pin_show_irq_mode ( struct pin_attribute *attr, char  *buf)
{
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_irq_handle_mode );
	return sprintf(buf, "%d\n", pin->irq_handle_mode );
}

/*
 *  Set irq handle mode for specified pin
 *
 */
static ssize_t 
pin_store_irq_mode( struct pin_attribute *attr, const char * buf, size_t count)
{
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_irq_handle_mode );
 	sscanf(buf, "%d", &pin->irq_handle_mode);

	//reset irq count to detect user suspend and kernel suspend
	if(pin->irq_handle_mode == IRQ_HANDLE_OFF)
		pin->irqs_during_suspend = 0;

	printk(KERN_INFO"USERPIN: setting irq handle mode of pin gpio %d to %d\n",
		pin->gpio, pin->irq_handle_mode);

	return count;
}


/*
 *  Show gpio direction 
 */
static int 
pin_show_direction ( struct pin_attribute *attr, char  *buf)
{
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_direction );
	return sprintf(buf, "%d\n", pin->direction );
}

/*
 *  Show  active level for specified pin
 */
static int 
pin_show_active ( struct pin_attribute *attr, char  *buf)
{
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_active );
	return sprintf(buf, "%d\n", pin->act_level );
}

/*
 *  Show gpio number
 */
static int 
pin_show_gpio ( struct pin_attribute *attr, char  *buf)
{
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_gpio );
	return sprintf(buf, "%d\n", pin->gpio );
}

/*
 *  Show current for specified pin
 */
static int 
pin_show_level ( struct pin_attribute *attr, char  *buf)
{
	int    val;
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_level );

	val = gpio_get_value ( pin->gpio );
	PDBG ( "get: gpio[%d] = %d\n", pin->gpio, val );
	if( val )
		return sprintf(buf, "1\n" );
	else
		return sprintf(buf, "0\n" );
}

/*
 *  Set level for specified pin
 */
#ifdef CONFIG_MODEM_POWER_ON_NOTIFY
RAW_NOTIFIER_HEAD(modem_power_on_notifier_list);
EXPORT_SYMBOL(modem_power_on_notifier_list);
#endif

static ssize_t 
pin_store_level( struct pin_attribute *attr, const char * buf, size_t count)
{
	int i = 0, len,  val = -1;
	struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_level );

	if( pin->options & PIN_READ_ONLY )
		return count;  // just ignore writes 

	/* skip leading white spaces */
	while( i < count && isspace(buf[i])) {
		i++;
	}

	len = count - i;
	if( len >= 1 && strncmp( buf+i, "1", 1) == 0) {
		val = 1;
		goto set;
	}

	if( len >= 1 && strncmp( buf+i, "0", 1) == 0) {
		val = 0;
		goto set;
	}

	if( len >= 4 && strncmp( buf+i, "high", 4) == 0) {
		val = 1;
		goto set;
	}

	if( len >= 3 && strncmp( buf+i, "low", 3) == 0) 
	{
		val = 0;
		goto set;
	}

	return count;

set:
	PDBG ("set: gpio[%d] = %d\n", pin->gpio, val );
#ifdef CONFIG_MODEM_POWER_ON_NOTIFY
	if (pin->gpio == MODEM_POWER_ON_GPIO)
		raw_notifier_call_chain(&modem_power_on_notifier_list,
					val, NULL);
#endif
	gpio_set_value ( pin->gpio, val );
	return count;
}

static irqreturn_t pin_irq_handler(int irq, void *data) {
	int i, j;
	struct platform_device *pdev = data;
	struct gpio_pin_dev_ctxt *dev_ctxt;
	struct gpio_pin_set *pset;

	printk("USERPIN_IRQ_HANDLER: Received irq=%d\n", irq);

	dev_ctxt = platform_get_drvdata ( pdev );
	if( dev_ctxt == NULL )
		return 0;

	for( i = 0; i< dev_ctxt->num_sets; i++ ) {
		pset = dev_ctxt->sets[i];
		for( j = 0; j < pset->num_pins; j++ ) {
			if (irq == gpio_to_irq(pset->pins[j].pin.gpio) &&
					pset->pins[j].pin.irq_handle_mode & IRQ_HANDLE_OFF) {

				pset->pins[j].pin.irqs_during_suspend++;

			}
		}
	}

	return IRQ_HANDLED;
}


/*
 *
 */
static void
pin_set_item_init (struct gpio_pin_set_item *psi, struct user_pin *up)
{
	psi->pin.name      = up->name;
	psi->pin.gpio      = up->gpio;
	psi->pin.options   = up->options;
	psi->pin.direction = up->direction;
	psi->pin.act_level = up->act_level;
	psi->pin.def_level = up->def_level;
	psi->pin.irq_config = up->irq_config;
	if(up->irq_config)
		psi->pin.irq_handler = pin_irq_handler;
	psi->pin.irq_handle_mode = up->irq_handle_mode;
	psi->pin.irqs_during_suspend = 0;

	// gpio attr
	psi->pin.attr_gpio.attr.name  = "gpio";
	psi->pin.attr_gpio.attr.mode  =  0444;
	psi->pin.attr_gpio.attr.owner =  THIS_MODULE;
	psi->pin.attr_gpio.show       =  pin_show_gpio;

	// level attr
	psi->pin.attr_level.attr.name  = "level";
	psi->pin.attr_level.attr.mode  =  0644;
	psi->pin.attr_level.attr.owner =  THIS_MODULE;
	psi->pin.attr_level.show       =  pin_show_level;
	psi->pin.attr_level.store      =  pin_store_level;

	// active attr
	psi->pin.attr_active.attr.name = "active";
	psi->pin.attr_active.attr.mode =  0444;
	psi->pin.attr_active.attr.owner=  THIS_MODULE;
	psi->pin.attr_active.show = pin_show_active;

	// direction
	psi->pin.attr_direction.attr.name  = "direction";
	psi->pin.attr_direction.attr.mode  =  0444;
	psi->pin.attr_direction.attr.owner =  THIS_MODULE;
	psi->pin.attr_direction.show       = pin_show_direction;
	
	// irq handle mode 
	psi->pin.attr_irq_handle_mode.attr.name  = "irq_handle_mode";
	psi->pin.attr_irq_handle_mode.attr.mode  =  0644;
	psi->pin.attr_irq_handle_mode.attr.owner =  THIS_MODULE;
	psi->pin.attr_irq_handle_mode.show       =  pin_show_irq_mode;
	psi->pin.attr_irq_handle_mode.store      =  pin_store_irq_mode;

	// setup attr pointer array 
	psi->pin.attr_ptr_arr[0] = &psi->pin.attr_gpio.attr;
	psi->pin.attr_ptr_arr[1] = &psi->pin.attr_level.attr;
	psi->pin.attr_ptr_arr[2] = &psi->pin.attr_active.attr;
	psi->pin.attr_ptr_arr[3] = &psi->pin.attr_direction.attr;
	if(up->irq_config) {
		psi->pin.attr_ptr_arr[4] = &psi->pin.attr_irq_handle_mode.attr;
		psi->pin.attr_ptr_arr[5] = NULL;
	} else {
		psi->pin.attr_ptr_arr[4] = NULL;
	}

	// setup  attribute group
	psi->attr_grp.name  = psi->pin.name;
	psi->attr_grp.attrs = psi->pin.attr_ptr_arr;

	return;
}

/*
 *
 */
static struct gpio_pin_set *
pin_set_alloc ( struct user_pin_set *ups )
{
	int i;
	struct gpio_pin_set *gps = NULL;

	gps = kzalloc( sizeof(struct gpio_pin_set) +  
                       ups->num_pins * sizeof(struct gpio_pin_set_item), 
	               GFP_KERNEL);
	if( gps == NULL )
		return NULL;

	gps->num_pins  = ups->num_pins;
	gps->set_name  = ups->set_name;

	for( i = 0; i < gps->num_pins; i++ ) {
		pin_set_item_init ( &gps->pins[i], &ups->pins[i] );
	}
 
	return gps;
}

/*
 *   Registers specified pin set
 */
static int 
pin_set_register( struct gpio_pin_set *s, struct platform_device *pdev)
{   
	int rc, i;

	if( s == NULL )
		return -EINVAL;

	s->kobj.parent = &pins_subsys.kobj;
	s->kobj.ktype  = &ktype_pin;
	kobject_set_name ( &s->kobj, s->set_name );
	rc = kobject_register ( &s->kobj );
	if( rc ) {
		printk ( KERN_ERR "Failed to register kobject (%s)\n", 
		         s->set_name);
		return -ENODEV;
	}	

	/* for all pins */
	for ( i = 0; i < s->num_pins; i++ ) 
	{
		rc = gpio_request ( s->pins[i].pin.gpio, "gpio" );
		if( rc  ) {
			printk ( KERN_ERR "Failed to request gpio (%d)\n", 
			         s->pins[i].pin.gpio );
			continue;
		}

		if(s->pins[i].pin.irq_handler != NULL)
		{
			printk("USERPINS: Configuring irq for gpio=%d\n",
				s->pins[i].pin.gpio);

			rc = request_irq(gpio_to_irq(s->pins[i].pin.gpio), s->pins[i].pin.irq_handler,
					s->pins[i].pin.irq_config, "userpins", pdev);

			if(rc)
			{
				printk("USERPINS: Failed to request irq!\n");
				continue;
			}
		}
 
		if( s->pins[i].pin.direction != -1 ) 
		{   // direction is set
			if( s->pins[i].pin.direction == 0 ) 
			{   // an output
				/* A setting of def_level == -1 means that we
				 * keep the current level of the GPIO.
				 * Otherwise we set def_level.
				 */
				int level = (-1 == s->pins[i].pin.def_level) ?
						gpio_get_value(s->pins[i].pin.gpio) :
						s->pins[i].pin.def_level;

				gpio_direction_output( s->pins[i].pin.gpio, level );
			}
			else
			{   // an input
				gpio_direction_input ( s->pins[i].pin.gpio );
			}
		}

		// create attribute group
		rc = sysfs_create_group( &s->kobj, &s->pins[i].attr_grp );
		if( rc ) {
			printk ( KERN_ERR "Failed to create sysfs attr group (%s)\n",
			         s->pins[i].pin.name );
		}
	}
       
	return 0;
}

static void 
pin_set_unregister( struct gpio_pin_set *s )
{
	int i;

	if( s == NULL )
		return;

	/* for all pins */
	for ( i = 0; s->num_pins; i++ ) 
	{
		if(s->pins[i].pin.irq_handler != NULL)
		{
			free_irq(gpio_to_irq(s->pins[i].pin.gpio), NULL);
		}
	
		sysfs_remove_group( &s->kobj, &s->pins[i].attr_grp );
		gpio_free ( s->pins[i].pin.gpio );
	}
	kobject_unregister ( &s->kobj );
	kfree ( s );
}

static int 
user_pins_probe  (struct platform_device *pdev)
{
	int i, rc;
	struct user_pins_platform_data *pdata;
	struct gpio_pin_dev_ctxt *dev_ctxt;
	
	pdata = pdev->dev.platform_data;
	if( pdata == NULL ) {
		return -ENODEV; 
	}

	dev_ctxt = kzalloc ( sizeof (struct gpio_pin_dev_ctxt) +
	                     pdata->num_sets * sizeof(struct gpio_pin_set *), 
	                     GFP_KERNEL );
	if( dev_ctxt == NULL ) {
		return -ENOMEM; 
	}
	dev_ctxt->num_sets = pdata->num_sets;
	spin_lock_init(&dev_ctxt->suspend_lock);

	for ( i = 0; i < dev_ctxt->num_sets; i++ ) {
		dev_ctxt->sets[i] = pin_set_alloc ( pdata->sets + i );
		if( dev_ctxt->sets[i] == NULL ) {
			printk ( KERN_ERR "Failed to init pin set '%s'\n", 
			         pdata->sets[i].set_name );
			continue;
		}
		rc = pin_set_register( dev_ctxt->sets[i], pdev );
		if( rc ) {
			printk ( KERN_ERR "Failed to register pin set '%s'\n",
			         pdata->sets[i].set_name );
		}
	}
	
	dev_set_drvdata ( &pdev->dev, dev_ctxt );

	return 0;
}

/*
 *
 */
static int 
user_pins_remove (struct platform_device *pdev)
{
	int i;
	struct gpio_pin_dev_ctxt *dev_ctxt;

	dev_ctxt = dev_get_drvdata ( &pdev->dev );
	if( dev_ctxt == NULL )
		return 0;

	for( i = 0; i < dev_ctxt->num_sets; i++ ) {
		pin_set_unregister ( dev_ctxt->sets[i] );
	}
	
	dev_set_drvdata ( &pdev->dev, NULL );
	kfree ( dev_ctxt );

	return 0;
}

#ifdef CONFIG_PM

/*
 *
 */
static int 
user_pins_suspend(struct platform_device *pdev, pm_message_t state)
{
	int i, j, irq;
	struct gpio_pin_dev_ctxt *dev_ctxt;
	struct gpio_pin_set      *pset;
	unsigned long flags;

	dev_ctxt = platform_get_drvdata ( pdev );
	if( dev_ctxt == NULL )
		return 0;

	spin_lock_irqsave ( &dev_ctxt->suspend_lock, flags );

	/*
 	 * The first loop is to check for any pending interrupts from
 	 * the time starting IRQ_HANDLE_OFF is set (from user space).
 	 * If so, fail the suspend, and let the system to go back up
 	 * to userspace.
 	 */

	for( i = 0; i < dev_ctxt->num_sets; i++ ) {
		pset = dev_ctxt->sets[i];
		for( j = 0; j < pset->num_pins; j++ ) {

			if( pset->pins[j].pin.irq_handle_mode & IRQ_HANDLE_OFF &&
			    pset->pins[j].pin.irqs_during_suspend != 0 ) {

				pset->pins[j].pin.irqs_during_suspend = 0;
				spin_unlock_irqrestore ( &dev_ctxt->suspend_lock, flags );

				printk(KERN_INFO"%s: not suspending due to pending irqs for gpio %d\n",
					__func__, pset->pins[j].pin.gpio);

				return -EBUSY;
			}
		}
	}

	for( i = 0; i < dev_ctxt->num_sets; i++ ) {
		pset = dev_ctxt->sets[i];
		for( j = 0; j < pset->num_pins; j++ ) {
			if( pset->pins[j].pin.options & PIN_WAKEUP_SOURCE ) {
				irq = gpio_to_irq(pset->pins[j].pin.gpio );

				if( pset->pins[i].pin.irq_handler != NULL ) {
					disable_irq(gpio_to_irq(pset->pins[j].pin.gpio));
				}
				enable_irq_wake(irq);
			}
		}
	}

	spin_unlock_irqrestore ( &dev_ctxt->suspend_lock, flags );


	return 0;
}

/*
 *
 */
static int 
user_pins_resume(struct platform_device *pdev)
{
	int i, j;
	struct gpio_pin_dev_ctxt *dev_ctxt;
	struct gpio_pin_set      *pset;

	dev_ctxt = platform_get_drvdata ( pdev );
	if( dev_ctxt == NULL )
		return 0;

	for( i = 0; i < dev_ctxt->num_sets; i++ ) {
		pset = dev_ctxt->sets[i];
		for( j = 0; j < pset->num_pins; j++ ) {
			if( pset->pins[j].pin.options & PIN_WAKEUP_SOURCE ) {
				int irq = gpio_to_irq(pset->pins[j].pin.gpio );
				disable_irq_wake(irq);
				if( pset->pins[i].pin.irq_handler != NULL ) {
					enable_irq(gpio_to_irq(pset->pins[j].pin.gpio));
				}
			}
			pset->pins[j].pin.irqs_during_suspend = 0;
		}
	}

	return 0;
}

#else

#define user_pins_suspend  NULL
#define user_pins_resume   NULL

#endif

static struct platform_driver user_pins_driver = {
	.driver		= {
		.name	= "user-pins",
	},
	.probe		= user_pins_probe,
	.remove		= __devexit_p(user_pins_remove),
	.suspend	= user_pins_suspend,
	.resume		= user_pins_resume,
};

static int __init 
user_pins_init(void)
{
	int rc;
	
	/* register pins subsystem */
	kobj_set_kset_s(&pins_subsys, user_hw_subsys);
	rc = subsystem_register ( &pins_subsys );
	if( rc ) {
		return -ENODEV;
	}
	
	/* register pins platform device */
	rc = platform_driver_register ( &user_pins_driver );
	if( rc ) {
		subsystem_unregister (&pins_subsys);
	}
	
	return rc; 
}

static void __exit   
user_pins_exit(void)
{
    platform_driver_unregister ( &user_pins_driver );
    subsystem_unregister       ( &pins_subsys );
}


module_init(user_pins_init);
module_exit(user_pins_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");


