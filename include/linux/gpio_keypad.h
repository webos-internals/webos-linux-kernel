#ifndef _GPIO_KEYPAD_H
#define _GPIO_KEYPAD_H

typedef enum
{
	GPIO_KEYPAD_CFG_ROW_IS_OUTPUT = 0,
	GPIO_KEYPAD_CFG_COL_IS_OUTPUT = 1
} GPIO_KEYPAD_CFG;

/* gpio keypad platform data structure */
struct gpio_kp_config {
	int  input_num;  // number of input gpios in matrix
	int  output_num; // number of output gpios in matrix
	int *inputs;     // input gpios 
	int *outputs;    // output gpios
	int  keymap_cfg; // GPIO_KEYPAD_CFG
 	int *keymap;     // keymap 
 	int  rep_period; // repeat period (msec) 
 	int  rep_delay;  // repeat delay  (msec)
	int  gpio_delay; // gpio select delay (usec)
	int  debounce;   // debounce interval (msec)
	int  wakeup_row; // single wakeup row is supported
	int  wakeup_mask;// 
	void (*kp_remux_func)(void);
	void (*kp_gpio_print_func)(void);
	int   key_prox_timeout; // key proximity timeout
    int   key_prox_width;   // key proximity width
    void *key_prox_map;     // key proximity map

};

#endif
