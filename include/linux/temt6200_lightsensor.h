#ifndef _TEMT6200_LIGHTSENSOR_H
#define _TEMT6200_LIGHTSENSOR_H

#define  TEMT6200_I2C_DEVICE   "TEMT6200"
#define  TEMT6200_I2C_DRIVER   "TEMT6200"

#define TEMT6200_DRIVER    "temt6200_light"

/* temt6200 lightsensor platform data structure */
struct temt6200_platform_data {
    char *desc;       // device name
    int   channel;    // madc channel number
    int   average;    // enable averaging 
    void  (*enable_lgt)(int enable);
};

#endif // TEMT6200 
