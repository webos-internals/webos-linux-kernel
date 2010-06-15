#ifndef __KXSD9_ACCELEROMETER_PLATFORM_H__ 
#define __KXSD9_ACCELEROMETER_PLATFORM_H__  

#define ACCELEROMETER_DEVICE		"kxsd9_accelerometer"
#define ACCELEROMETER_DRIVER		"kxsd9_accelerometer"
#define ACCELEROMETER_I2C_ADDRESS	0x18

struct kxsd9_platform_data {
	char   *dev_name;
	int * xyz_translation_map;
};
#endif /* __KXSD9_ACCELEROMETER_PLATFORM_H__  */ 
