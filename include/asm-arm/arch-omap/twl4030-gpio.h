/*
 * twl4030-gpio.h - header for TWL4030-GPIO
 *
 */

#ifndef __TWL4030_GPIO_H_
#define __TWL4030_GPIO_H_

int twl4030_request_gpio(int gpio);
int twl4030_free_gpio(int gpio);
int twl4030_set_gpio_direction(int gpio, int is_input);
int twl4030_get_gpio_datain(int gpio);
int twl4030_set_gpio_dataout(int gpio, int enable);
int twl4030_set_gpio_pull(int gpio, int pull_dircn);
int twl4030_set_gpio_edge_ctrl(int gpio, int edge);
int twl4030_set_gpio_debounce(int gpio, int enable);
int twl4030_set_gpio_card_detect(int gpio, int enable);

#endif 
