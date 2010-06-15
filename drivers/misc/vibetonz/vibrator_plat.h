#ifndef _VIBRATOR_PLAT_H
#define _VIBRATOR_PLAT_H

#include <linux/types.h>
#include <linux/vibrator.h>


int plat_vibrator_register(struct vibrator_platform_data* i_p_data);
int plat_vibrator_init(void);
int plat_vibrator_deinit(void);
int plat_vibrator_set_direction(int dir_control);
int plat_vibrator_set_duty_cycle(unsigned int duty_cycle);
int plat_vibrator_enable(int enable);

#endif  // _VIBRATOR_PLAT_H
