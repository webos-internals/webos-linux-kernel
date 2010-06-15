  zreladdr-y		:= 0x10008000
params_phys-y		:= 0x10000100
initrd_phys-y		:= 0x10800000

# 7x25 based cpus start sdram at 0
  zreladdr-$(CONFIG_ARCH_MSM7X25)	:= 0x00208000
params_phys-$(CONFIG_ARCH_MSM7X25)	:= 0x00200100
initrd_phys-$(CONFIG_ARCH_MSM7X25)	:= 0x00a00000
