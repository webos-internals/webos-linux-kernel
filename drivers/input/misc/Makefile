#
# Makefile for the input misc drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_INPUT_SPARCSPKR)		+= sparcspkr.o
obj-$(CONFIG_INPUT_PCSPKR)		+= pcspkr.o
obj-$(CONFIG_INPUT_M68K_BEEP)		+= m68kspkr.o
obj-$(CONFIG_INPUT_IXP4XX_BEEPER)	+= ixp4xx-beeper.o
obj-$(CONFIG_INPUT_COBALT_BTNS)		+= cobalt_btns.o
obj-$(CONFIG_INPUT_WISTRON_BTNS)	+= wistron_btns.o
obj-$(CONFIG_INPUT_ATLAS_BTNS)		+= atlas_btns.o
obj-$(CONFIG_INPUT_ATI_REMOTE)		+= ati_remote.o
obj-$(CONFIG_INPUT_ATI_REMOTE2)		+= ati_remote2.o
obj-$(CONFIG_INPUT_KEYSPAN_REMOTE)	+= keyspan_remote.o
obj-$(CONFIG_INPUT_POWERMATE)		+= powermate.o
obj-$(CONFIG_INPUT_YEALINK)		+= yealink.o
obj-$(CONFIG_HP_SDC_RTC)		+= hp_sdc_rtc.o
obj-$(CONFIG_INPUT_UINPUT)		+= uinput.o

obj-$(CONFIG_HSDL9100_PROX_SENSOR)	+= hsdl9100_proxsensor.o

hsdl9100_proxsensor-objs := hsdl9100_prox_sense.o 
hsdl9100_proxsensor-$(CONFIG_ARCH_OMAP) += hsdl9100_prox_sense_omap.o

obj-$(CONFIG_LIGHTSENSOR_AMBIENT6200)    += temt6200_lightsensor.o
