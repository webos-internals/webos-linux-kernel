# Makefile for OMAP1/2 camera driver

obj-y += isp/

obj-y +=gci/

ifdef CONFIG_ARCH_OMAP1
obj-$(CONFIG_VIDEO_OMAP_CAMERA) += omapcamera.o
obj-$(CONFIG_VIDEO_CAMERA_SENSOR_OV9640) += sensor_ov9640.o

objs-y$(CONFIG_ARCH_OMAP16XX) += omap16xxcam.o camera_core.o
objs-y$(CONFIG_MACH_OMAP_H3) += h3_sensor_power.o

else
obj-$(CONFIG_VIDEO_OMAP24XX_VIDEOLIB) += omap24xxlib.o 

ifndef CONFIG_ARCH_OMAP2430
obj-$(CONFIG_VIDEO_OMAP24XX_CAMERA) += sensor_ov9640.o h4_sensor_power.o 
endif
obj-$(CONFIG_VIDEO_OMAP_SENSOR_MT9D111) += sensor_ex3691.o 
obj-$(CONFIG_VIDEO_OMAP_SENSOR_MT9D112) += sensor_mt9d112.o
obj-$(CONFIG_VIDEO_OMAP_SENSOR_MT9T012) += sensor_mt9t012.o 
obj-$(CONFIG_VIDEO_OMAP_SENSOR_OV3640) += sensor_ov3640.o

obj-$(CONFIG_VIDEO_OMAP_SENSOR_MT9P012) += sensor_mt9p012.o
obj-$(CONFIG_VIDEO_OMAP_AF_D88) += af_d88.o

obj-$(CONFIG_VIDEO_OMAP24XX_CAMERA) += omap24xxcam.o
obj-$(CONFIG_VIDEO_OMAP34XX_CAMERA) += omap34xxcam.o

obj-$(CONFIG_VIDEO_OMAP24XX_VIDEOOUT) += omap24xxvout.o
endif

omapcamera-objs := $(objs-yy)

EXTRA_CFLAGS = -I$(src)/..
