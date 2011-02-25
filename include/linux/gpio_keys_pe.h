#ifndef _GPIO_KEYS_H
#define _GPIO_KEYS_H

struct gpio_keys_button {
	/* Configuration parameters */
	int code;		/* input event code (KEY_*, SW_*) */
	int gpio;
	int active_low;
	char *desc;
	int debounce;		/* debounce interval (msec) */
	int type;		/* input event type (EV_KEY, EV_SW) */
	int wakeup;		/* configure the button as a wake-up source */
	int options;		/* device specific options */
#ifdef CONFIG_ARCH_OMAP3
	char *pin;              /* pin mux name */
#endif
};

struct gpio_keys_platform_data {
	struct gpio_keys_button *buttons;
	int nbuttons;
};

#define OPT_REBOOT_TRIGGER       (1 << 0)  // can be reboot trigger  
#define OPT_REBOOT_TRIGGER_LEVEL (0)       // triggered by level
#define OPT_REBOOT_TRIGGER_EDGE  (1 << 1)  // triggered by edge

#define OPT_CONSOLE_TRIGGER      (1 << 2)  // can be console switch trigger


#endif
