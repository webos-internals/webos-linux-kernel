#ifndef CAMERA_MT9P013_H
#define CAMERA_MT9P013_H

struct mt9p013_platform_data {
	int    (*init)(void);
	int    (*deinit)(void);
	int    (*power_shutdown)(void);
	int    (*power_resume)(void);
};

int32_t mt9p013_enable_flash(uint8_t enable);

#endif

