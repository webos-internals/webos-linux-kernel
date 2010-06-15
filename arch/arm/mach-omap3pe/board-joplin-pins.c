/*
 * arch/arm/mach-omap2/board-joplin-gpio.c
 *
 * Copyright (C) 2007 Palm, Inc.
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
#include <linux/platform_device.h>

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

#if   defined(CONFIG_MACH_BRISKET)

#define BT_RESET_GPIO                134
#define BT_RESET_GPIO_PIN_MODE        -1

#define BT_WAKE_GPIO                 136
#define BT_WAKE_GPIO_PIN_MODE         -1

#define BT_HOST_WAKE_GPIO             94
#define BT_HOST_WAKE_GPIO_PIN_MODE    -1

#define MODEM_POWER_ON               152  // OUT
#define MODEM_POWER_ON_PIN_MODE       -1

#define MODEM_BOOT_MODE              153  // OUT
#define MODEM_BOOT_MODE_PIN_MODE      -1

#define MODEM_WAKEUP_MODEM            87  // OUT
#define MODEM_WAKEUP_MODEM_PIN_MODE   -1

#define MODEM_WAKEUP_APP              49  // IN
#define MODEM_WAKEUP_APP_PIN_MODE     -1

//#define MODEM_WAKE_USB               155 // IN
//#define MODEM_WAKE_USB_PIN_MODE       -1

#elif defined(CONFIG_MACH_FLANK) || defined(CONFIG_MACH_SIRLOIN)

#define BT_RESET_GPIO                178 
#define BT_RESET_GPIO_PIN_MODE       AA3_BT_nRST

#define BT_WAKE_GPIO                 177 
#define BT_WAKE_GPIO_PIN_MODE        AB2_BT_nWAKE

#define BT_HOST_WAKE_GPIO            176 
#define BT_HOST_WAKE_GPIO_PIN_MODE   AB1_BT_HOST_WAKE

#define MODEM_POWER_ON               153 // OUT
#define MODEM_POWER_ON_PIN_MODE      AD1_3430_GPIO153

#define MODEM_BOOT_MODE              154 // OUT
#define MODEM_BOOT_MODE_PIN_MODE     AD2_3430_GPIO154

#define MODEM_WAKEUP_MODEM           157 // OUT
#define MODEM_WAKEUP_MODEM_PIN_MODE  AA21_3430_GPIO157
 
#define MODEM_WAKEUP_APP             152 // IN
#define MODEM_WAKEUP_APP_PIN_MODE    AE1_3430_GPIO152

#define MODEM_WAKE_USB               155 // IN
#define MODEM_WAKE_USB_PIN_MODE      AC1_3430_GPIO155

#define AUDIO_PCM_R_TO_BT_CLK             41
#define AUDIO_PCM_R_TO_BT_CLK_PIN_MODE    AB18_3430_GPIO41
#define AUDIO_PCM_R_TO_BT_DATA            42
#define AUDIO_PCM_R_TO_BT_DATA_PIN_MODE   AC19_3430_GPIO42
#define AUDIO_PCM_REMOTE_MASTER           43
#define AUDIO_PCM_REMOTE_MASTER_PIN_MODE  AB19_3430_GPIO43

#else

#error "No support for your MACH yet"

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
    int     direction;
    int     act_level;
    int     def_level;
    int     pin_mode;
    const char * name;
    struct  pin_attribute attr_gpio;
    struct  pin_attribute attr_level;
    struct  pin_attribute attr_active;
    struct  pin_attribute attr_direction;
};

struct gpio_pin_set_item {
    struct gpio_pin        *pin;
    struct attribute_group *pin_grp;
};

struct gpio_pin_set {
    const  char *name;
    struct kobject kobj;
    struct gpio_pin_set_item *pins;
};

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
static ssize_t 
pin_store_level( struct pin_attribute *attr, const char * buf, size_t count)
{
    int i = 0, len,  val = -1;
    struct gpio_pin *pin = container_of(attr, struct gpio_pin, attr_level );
    
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
    gpio_set_value ( pin->gpio, val );
    return count;
}

#if defined(CONFIG_MACH_FLANK) || defined(CONFIG_MACH_SIRLOIN)
/*
*  Audio
*/

/*
 *   level shifting pins - connect BT to modem directly - initally disabled
*/
static struct gpio_pin audio_a = {
   .gpio       =  AUDIO_PCM_R_TO_BT_CLK,  //
    .act_level  =  1,  // active high
    .direction  =  0,  // an output
    .def_level  =  0,  // initialy low
    .pin_mode   =  AUDIO_PCM_R_TO_BT_CLK_PIN_MODE,
    .name       = "AUDIO_PCM_R_TO_BT_CLK",
    .attr_gpio = {
        .attr  = {
            .name  = "gpio",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_gpio,
    },
    .attr_level = {
        .attr  = {
            .name  = "level",
            .mode  =  0644,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_level,
        .store  = pin_store_level,
    },
    .attr_active = {
        .attr   = {
            .name  = "active",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_active,
    },
    .attr_direction = {
        .attr   = {
            .name  = "direction",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_direction,
    }
};

static struct attribute *audio_a_attrs[] = {
    &audio_a.attr_gpio.attr,
    &audio_a.attr_level.attr,
    &audio_a.attr_active.attr,
    &audio_a.attr_direction.attr,
    NULL
};

struct attribute_group  audio_a_grp = {
    .name  = "AUDIO_PCM_R_TO_BT_CLK",
    .attrs =  audio_a_attrs,
};






static struct gpio_pin audio_b = {
    .gpio       =  AUDIO_PCM_R_TO_BT_DATA,  //
    .act_level  =  1,  // active high
    .direction  =  0,  // an output
    .def_level  =  1,  // initialy high
    .pin_mode   =  AUDIO_PCM_R_TO_BT_DATA_PIN_MODE,
    .name       = "AUDIO_PCM_R_TO_BT_DATA",
    .attr_gpio = {
        .attr  = {
            .name  = "gpio",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_gpio,
    },
    .attr_level = {
        .attr  = {
            .name  = "level",
            .mode  =  0644,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_level,
        .store  = pin_store_level,
    },
    .attr_active = {
        .attr   = {
            .name  = "active",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_active,
    },
    .attr_direction = {
        .attr   = {
            .name  = "direction",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_direction,
    }
};

static struct attribute *audio_b_attrs[] = {
    &audio_b.attr_gpio.attr,
    &audio_b.attr_level.attr,
    &audio_b.attr_active.attr,
    &audio_b.attr_direction.attr,
    NULL
};

struct attribute_group  audio_b_grp = {
    .name  = "AUDIO_PCM_R_TO_BT_DATA",
    .attrs =  audio_b_attrs,
};

static struct gpio_pin audio_c = {
    .gpio       =  AUDIO_PCM_REMOTE_MASTER,  //
    .act_level  =  1,  // active high
    .direction  =  0,  // an output
    .def_level  =  1,  // initialy high
    .pin_mode   =  AUDIO_PCM_REMOTE_MASTER_PIN_MODE,
    .name       = "AUDIO_PCM_REMOTE_MASTER",
    .attr_gpio = {
        .attr  = {
            .name  = "gpio",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_gpio,
    },
    .attr_level = {
        .attr  = {
            .name  = "level",
            .mode  =  0644,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_level,
        .store  = pin_store_level,
    },
    .attr_active = {
        .attr   = {
            .name  = "active",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_active,
    },
    .attr_direction = {
        .attr   = {
            .name  = "direction",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_direction,
    }
};

static struct attribute *audio_c_attrs[] = {
    &audio_c.attr_gpio.attr,
    &audio_c.attr_level.attr,
    &audio_c.attr_active.attr,
    &audio_c.attr_direction.attr,
    NULL
};

struct attribute_group  audio_c_grp = {
    .name  = "AUDIO_PCM_REMOTE_MASTER",
    .attrs =  audio_c_attrs,
};


struct gpio_pin_set_item audio_pin_array[] = {
    { &audio_a,   &audio_a_grp   },
    { &audio_b,   &audio_b_grp   },
    { &audio_c,   &audio_c_grp   },
    { NULL, NULL},
};

struct gpio_pin_set audio_pins = {
    .name = "audio",
    .pins =  audio_pin_array,
};
#endif


/*
 *  BT reset gpio
 */
static struct gpio_pin bt_reset = {
    .gpio      = BT_RESET_GPIO,   
    .act_level = 0,     // active Low
    .direction = 0,     // an output
    .def_level = 0,     // default level (low)
    .pin_mode  = BT_RESET_GPIO_PIN_MODE,
    .name      = "reset",
    .attr_gpio = {
        .attr  = {
            .name  = "gpio",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_gpio,
    },
    .attr_level = {
        .attr  = {
            .name  = "level",
            .mode  =  0644,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_level,
        .store  = pin_store_level,
    },
    .attr_active = {
        .attr   = {
            .name  = "active",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_active,
    },
    .attr_direction = {
        .attr   = {
            .name  = "direction",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_direction,
    }
};

static struct attribute *bt_reset_attrs[] = {
    &bt_reset.attr_gpio.attr,
    &bt_reset.attr_level.attr,
    &bt_reset.attr_active.attr,
    &bt_reset.attr_direction.attr,
    NULL
};

struct attribute_group  bt_reset_grp = {
    .name  = "reset",
    .attrs = bt_reset_attrs,
};

/*
 *  BT wake gpio
 */
static struct gpio_pin bt_wake  = {
    .gpio       = BT_WAKE_GPIO,  
    .act_level  = 0, // active Low
    .direction  = 0, // an output
    .def_level  = 0, // undefined
    .pin_mode   = BT_WAKE_GPIO_PIN_MODE,
    .name       = "wake",
    .attr_gpio = {
        .attr  = {
            .name  = "gpio",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_gpio,
    },
    .attr_level = {
        .attr  = {
            .name  = "level",
            .mode  =  0644,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_level,
        .store  = pin_store_level,
    },
    .attr_active = {
        .attr   = {
            .name  = "active",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_active,
    },
    .attr_direction = {
        .attr   = {
            .name  = "direction",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_direction,
    }
};

static struct attribute *bt_wake_attrs[] = {
    &bt_wake.attr_gpio.attr,
    &bt_wake.attr_level.attr,
    &bt_wake.attr_active.attr,
    &bt_wake.attr_direction.attr,
    NULL
};

struct attribute_group  bt_wake_grp = {
    .name  = "wake",
    .attrs = bt_wake_attrs,
};

/*
 *  BT wake host gpio
 */
static struct gpio_pin bt_host_wake  = {
    .gpio       =  BT_HOST_WAKE_GPIO,  //
    .act_level  =  1,  // active high
    .direction  =  1,  // an input
    .def_level  = -1,  // undefined
    .pin_mode   =  BT_HOST_WAKE_GPIO_PIN_MODE,
    .name       = "host_wake",
    .attr_gpio = {
        .attr  = {
            .name  = "gpio",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_gpio,
    },
    .attr_level = {
        .attr  = {
            .name  = "level",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_level,
        .store  = pin_store_level,
    },
    .attr_active = {
        .attr   = {
            .name  = "active",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_active,
    },
    .attr_direction = {
        .attr   = {
            .name  = "direction",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_direction,
    }
};

static struct attribute *bt_host_wake_attrs[] = {
    &bt_host_wake.attr_gpio.attr,
    &bt_host_wake.attr_level.attr,
    &bt_host_wake.attr_active.attr,
    &bt_host_wake.attr_direction.attr,
    NULL
};

struct attribute_group  bt_host_wake_grp = {
    .name  = "host_wake",
    .attrs =  bt_host_wake_attrs,
};

/*
 *  BT pins array
 */
struct gpio_pin_set_item bt_pin_array[] = {
    { &bt_reset,      &bt_reset_grp    },
    { &bt_wake,       &bt_wake_grp     },
    { &bt_host_wake,  &bt_host_wake_grp},
    { NULL, NULL},
};

struct gpio_pin_set bt_pins = {
    .name = "bt",
    .pins =  bt_pin_array,
};



/*
 *  Modem
 */
/*
 *   Modem power on pin
 */
static struct gpio_pin modem_power_on = {
    .gpio       =  MODEM_POWER_ON,  //
    .act_level  =  1,  // active high
    .direction  =  0,  // an output
    .def_level  =  0,  // initialy low
    .pin_mode   =  MODEM_POWER_ON_PIN_MODE,
    .name       = "power_on",
    .attr_gpio = {
        .attr  = {
            .name  = "gpio",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_gpio,
    },
    .attr_level = {
        .attr  = {
            .name  = "level",
            .mode  =  0644,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_level,
        .store  = pin_store_level,
    },
    .attr_active = {
        .attr   = {
            .name  = "active",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_active,
    },
    .attr_direction = {
        .attr   = {
            .name  = "direction",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_direction,
    }
};

static struct attribute *modem_power_on_attrs[] = {
    &modem_power_on.attr_gpio.attr,
    &modem_power_on.attr_level.attr,
    &modem_power_on.attr_active.attr,
    &modem_power_on.attr_direction.attr,
    NULL
};

struct attribute_group  modem_power_on_grp = {
    .name  = "power_on",
    .attrs =  modem_power_on_attrs,
};

/*
 *   Modem boot mode pin
 */
static struct gpio_pin modem_boot_mode = {
    .gpio       =  MODEM_BOOT_MODE,  //
    .act_level  =  -1,  // not applicable
    .direction  =   0,  // an output
    .def_level  =   0,  // initially low
    .pin_mode   =  MODEM_BOOT_MODE_PIN_MODE,
    .name       = "boot_mode",
    .attr_gpio = {
        .attr  = {
            .name  = "gpio",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_gpio,
    },
    .attr_level = {
        .attr  = {
            .name  = "level",
            .mode  =  0644,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_level,
        .store  = pin_store_level,
    },
    .attr_active = {
        .attr   = {
            .name  = "active",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_active,
    },
    .attr_direction = {
        .attr   = {
            .name  = "direction",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_direction,
    }
};

static struct attribute *modem_boot_mode_attrs[] = {
    &modem_boot_mode.attr_gpio.attr,
    &modem_boot_mode.attr_level.attr,
    &modem_boot_mode.attr_active.attr,
    &modem_boot_mode.attr_direction.attr,
    NULL
};

struct attribute_group  modem_boot_mode_grp = {
    .name  = "boot_mode",
    .attrs =  modem_boot_mode_attrs,
};

/*
 *   Modem app wakup pin
 */
static struct gpio_pin modem_wake_modem = {
    .gpio       =  MODEM_WAKEUP_MODEM,  //
    .act_level  =  1,  // active high
    .direction  =  0,  // an output
    .def_level  =  0,  // initialy low
    .pin_mode   =  MODEM_WAKEUP_MODEM_PIN_MODE,
    .name       =  "wakeup_modem",
    .attr_gpio = {
        .attr  = {
            .name  = "gpio",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_gpio,
    },
    .attr_level = {
        .attr  = {
            .name  = "level",
            .mode  =  0644,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_level,
        .store  = pin_store_level,
    },
    .attr_active = {
        .attr   = {
            .name  = "active",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_active,
    },
    .attr_direction = {
        .attr   = {
            .name  = "direction",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_direction,
    }
};

static struct attribute *modem_wake_modem_attrs[] = {
    &modem_wake_modem.attr_gpio.attr,
    &modem_wake_modem.attr_level.attr,
    &modem_wake_modem.attr_active.attr,
    &modem_wake_modem.attr_direction.attr,
    NULL
};

struct attribute_group  modem_wake_modem_grp = {
    .name  = "wakeup_modem",
    .attrs =  modem_wake_modem_attrs,
};

/*
 *   Modem wakeup app
 */
static struct gpio_pin modem_wake_app = {
    .gpio       =   MODEM_WAKEUP_APP,
    .act_level  =   1,  // active high
    .direction  =   1,  // an output
    .def_level  =  -1,  // unset
    .pin_mode   =   MODEM_WAKEUP_APP_PIN_MODE,
    .name       =   "wakeup_app",
    .attr_gpio = {
        .attr  = {
            .name  = "gpio",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_gpio,
    },
    .attr_level = {
        .attr  = {
            .name  = "level",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show   = pin_show_level,
        .store  = pin_store_level,
    },
    .attr_active = {
        .attr   = {
            .name  = "active",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_active,
    },
    .attr_direction = {
        .attr   = {
            .name  = "direction",
            .mode  =  0444,
            .owner =  THIS_MODULE,
         },
        .show = pin_show_direction,
    }
};

static struct attribute *modem_wake_app_attrs[] = {
    &modem_wake_app.attr_gpio.attr,
    &modem_wake_app.attr_level.attr,
    &modem_wake_app.attr_active.attr,
    &modem_wake_app.attr_direction.attr,
    NULL
};

struct attribute_group  modem_wake_app_grp = {
    .name  = "wakeup_app",
    .attrs =  modem_wake_app_attrs,
};

struct gpio_pin_set_item modem_pin_array[] = {
    { &modem_power_on,   &modem_power_on_grp   },
    { &modem_boot_mode,  &modem_boot_mode_grp  },
    { &modem_wake_modem, &modem_wake_modem_grp },
    { &modem_wake_app,   &modem_wake_app_grp   },
    { NULL, NULL},
};

struct gpio_pin_set modem_pins = {
    .name = "modem",
    .pins =  modem_pin_array,
};

/*
 *   Registers specified pin set
 */
static int pin_set_register( struct gpio_pin_set *s )
{   
    int rc, i;

	s->kobj.parent = &pins_subsys.kobj;
	s->kobj.ktype  = &ktype_pin;
    kobject_set_name ( &s->kobj, s->name );
    rc = kobject_register ( &s->kobj );

    /* for all pins */
    i = 0;
    for ( i = 0; s->pins[i].pin; i++ ) 
    {
        rc = gpio_request ( s->pins[i].pin->gpio, "gpio" );
        if( rc == 0 ) 
        {   // registereted OK
            if( s->pins[i].pin->direction != -1 ) 
            {   // direction is set
                if( s->pins[i].pin->direction == 0 ) 
                {   // an output
                    gpio_direction_output( s->pins[i].pin->gpio, 
                                           s->pins[i].pin->def_level );
                }
                else
                {   // an input
                    gpio_direction_input ( s->pins[i].pin->gpio );
                }
            }
            // and config it if needed
            if( s->pins[i].pin->pin_mode != -1 ) 
                omap_cfg_reg ( s->pins[i].pin->pin_mode );
                
            // register attributes
            rc = sysfs_create_group( &s->kobj, s->pins[i].pin_grp );
            if( rc != 0 ) 
            {
                gpio_free ( s->pins[i].pin->gpio );
                goto err;
            }
        }
    }
        
    return 0;
    
err:
    return -ENODEV;
}

int pin_get_gpio(char *name)
{
	int i;
	
	for ( i = 0; bt_pins.pins[i].pin; i++ ) 
	{
		if (strcmp(name, bt_pins.pins[i].pin->name) == 0)
		{
			return (bt_pins.pins[i].pin->gpio);
		}

	}

	for ( i = 0; modem_pins.pins[i].pin; i++ ) 
	{
		if (strcmp(name, modem_pins.pins[i].pin->name) == 0)
		{
			return (modem_pins.pins[i].pin->gpio);
		}
	}
	return(-1);
}
EXPORT_SYMBOL(pin_get_gpio);

/*
 *
 */
static int board_pins_probe  (struct platform_device *dev)
{
    int rc;

    rc = pin_set_register( &bt_pins );
    if( rc ) {
        printk ( KERN_ERR "Failed to register bt_pins\n" );
    }

    rc = pin_set_register( &modem_pins );
    if( rc ) {
        printk ( KERN_ERR "Failed to register modem_pins\n" );
    }

#if defined(CONFIG_MACH_FLANK) || defined(CONFIG_MACH_SIRLOIN)
    rc = pin_set_register( &audio_pins );
    if( rc ) {
        printk ( KERN_ERR "Failed to register audio_pins\n" );
    }
#endif

    return 0;
}

/*
 *
 */
static int board_pins_remove (struct platform_device *dev)
{
    return 0;
}

#ifdef CONFIG_PM

/*
 *
 */
static int board_pins_suspend(struct platform_device *dev, pm_message_t state)
{
    return 0;
}

/*
 *
 */
static int board_pins_resume(struct platform_device *dev)
{
    return 0;
}

#else

#define board_pins_suspend  NULL
#define board_pins_resume   NULL

#endif

static struct platform_driver board_pins_driver = {
	.driver		= {
		.name	= "pins",
	},
	.probe		= board_pins_probe,
	.remove		= __devexit_p(board_pins_remove),
	.suspend	= board_pins_suspend,
	.resume		= board_pins_resume,
};

static int __init board_pins_init(void)
{
    int rc;
    
    /* register pins subsystem */
	kobj_set_kset_s(&pins_subsys, user_hw_subsys);
    rc = subsystem_register ( &pins_subsys );
    if( rc ) {
        return -ENODEV;
    }
    
    /* register pins platform device */
	rc = platform_driver_register ( &board_pins_driver );
	if( rc ) {
	    subsystem_unregister (&pins_subsys);
	}

	return rc;    
}

static void __exit   board_pins_exit(void)
{
    platform_driver_unregister ( &board_pins_driver );
    subsystem_unregister       ( &pins_subsys );
}


module_init(board_pins_init);
module_exit(board_pins_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");


