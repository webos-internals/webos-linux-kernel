#ifndef _VIBRATOR_H
#define _VIBRATOR_H

#include <linux/types.h>

#define VIBE_DEVICE   "vibe"
#define VIBE_DRIVER   "vibe"

struct vibrator_platform_data {
	int vibrator_enable_gpio;
	void (*power) (int on);
};

#endif  // _VIBRATOR_H
