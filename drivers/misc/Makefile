#
# Makefile for misc devices that really don't fit anywhere else.
#

obj-$(CONFIG_IBM_ASM)		+= ibmasm/
obj-$(CONFIG_HDPU_FEATURES)	+= hdpuftrs/
obj-$(CONFIG_ATMEL_PWM)		+= atmel_pwm.o
obj-$(CONFIG_ATMEL_SSC)		+= atmel-ssc.o
obj-$(CONFIG_ATMEL_TCLIB)	+= atmel_tclib.o
obj-$(CONFIG_ICS932S401)	+= ics932s401.o
obj-$(CONFIG_LKDTM)		+= lkdtm.o
obj-$(CONFIG_TIFM_CORE)       	+= tifm_core.o
obj-$(CONFIG_TIFM_7XX1)       	+= tifm_7xx1.o
obj-$(CONFIG_PHANTOM)		+= phantom.o
obj-$(CONFIG_ANDROID_PMEM)	+= pmem.o
obj-$(CONFIG_ANDROID_PMEM_KAPI_TEST)	+= pmem_kernel_test.o
obj-$(CONFIG_SGI_IOC4)		+= ioc4.o
obj-$(CONFIG_ENCLOSURE_SERVICES) += enclosure.o
obj-$(CONFIG_KERNEL_DEBUGGER_CORE)	+= kernel_debugger.o
obj-$(CONFIG_KGDB_TESTS)	+= kgdbts.o
obj-$(CONFIG_SGI_XP)		+= sgi-xp/
obj-$(CONFIG_SGI_GRU)		+= sgi-gru/
obj-$(CONFIG_HP_ILO)		+= hpilo.o
obj-$(CONFIG_UID_STAT)		+= uid_stat.o
obj-$(CONFIG_C2PORT)		+= c2port/
obj-y				+= eeprom/
obj-$(CONFIG_MSM_720P_CORE)	+= video_core/720p/
obj-$(CONFIG_TSIF) += msm_tsif.o
obj-$(CONFIG_OMAP_VIBRATOR)     += vibetonz/
obj-$(CONFIG_MSM_VIBRATOR)      += vibetonz/
msm_tsif-objs := tsif.o

obj-$(CONFIG_TSIF_CHRDEV) += tsif_chrdev.o
obj-$(CONFIG_PMIC8058_PWM) += pmic8058-pwm.o
obj-$(CONFIG_PMIC8058_NFC) += pmic8058-nfc.o

obj-$(CONFIG_NDUID)		+= nduid.o
obj-$(CONFIG_USER_PINS)		+= user-pins.o
obj-$(CONFIG_PINMUX)		+= pinmux.o
obj-$(CONFIG_KERNEL_LOG)	+= klog.o
obj-$(CONFIG_HRES_COUNTER)	+= hres_counter.o
obj-$(CONFIG_A6)		+= a6/
obj-$(CONFIG_LOW_MEMORY_NOTIFY) += lowmemnotify.o

obj-$(CONFIG_HSUART) 			+= hsuart.o
obj-$(CONFIG_BLUETOOTH_POWER_STATE) += bluetooth-power-pe.o
obj-$(CONFIG_CHARGER_SMB339)	+= charger-smb339.o

obj-$(CONFIG_HARDKEY_DIAG)    += hardkey_diag.o

