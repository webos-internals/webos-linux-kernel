#ifndef _MAXIM7359_KEYPAD_H
#define _MAXIM7359_KEYPAD_H

#define  MAXIM7359_I2C_DEVICE   "MAXIM7359"
#define  MAXIM7359_I2C_DRIVER   "MAXIM7359"

/* maxim7359 keypad platform data structure */
struct maxim7359_platform_data {
    char *dev_name;   // device name
    int   row_num;    // number of rows in matrix
    int   col_num;    // number of cols in matrix
    int  *keymap;     // keymap 
    int   key_prox_timeout; // key proximity timeout
    int   key_prox_width;   // key proximity width
    void *key_prox_map;     // key proximity map
    int   rep_period; // repeat period (msec) 
    int   rep_delay;  // repeat delay  (msec)
    int   debounce;   // debounce interval (msec)
    int   wakeup_en;  // wake up source enable/disable
};

#endif // MAXIM7359
