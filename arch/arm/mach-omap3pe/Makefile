#
# Makefile for the linux kernel.
#
# Common support

obj-y := irq.o id.o io.o memory.o mux.o devices.o \
	 serial.o gpmc.o 

# DSP
obj-$(CONFIG_OMAP_MMU_FWK)	+= mmu_mach.o
obj-$(CONFIG_OMAP_MBOX_FWK)	+= mailbox_mach.o
mailbox_mach-objs		:= mailbox.o
mmu_mach-objs			:= mmu.o

# Specific board support
ifeq ($(CONFIG_ARCH_OMAP34XX),y)
obj-y					+= \
					   clock.o \
					   sram-fn.o \
					   sram_idle.o \
					   \
					   prcm.o \
					   prcm_clksel.o \
					   prcm_clk.o \
					   prcm_pwr.o \
					   prcm_mpu_core_ctl.o \
					   prcm_opp.o \
					   prcm_slpwk.o \
					   prcm_regs.o \
					   \
					   smartreflex.o \
					   \
					   context.o \
					   scratchpad.o \
					   pm.o \
					   pm_sysfs.o \
					   sleep.o \
					   \
					   prcm_util.o \
					   resource.o \
					   constraint.o

obj-$(CONFIG_CPU_IDLE)			+= pm_cpuidle.o
endif

obj-$(CONFIG_MACH_OMAP_GENERIC)		+= board-generic.o

# TI SDP reference board
obj-$(CONFIG_MACH_OMAP_3430SDP)		+= board-3430sdp.o \
					   board-3430sdp-flash.o \
					   board-3430sdp-usb.o \
					   board-3430sdp-hsmmc.o

# Sirloin (OMAP3430)
ifeq ($(CONFIG_MACH_SIRLOIN),y)

obj-y					+= board-sirloin-3430.o \
					   board-sirloin-3430-pwr.o \
					   board-sirloin-3430-pm.o \
					   boot-wall.o \
					   board-nduid.o

#obj-$(CONFIG_SND) 			+= board-sirloin-hs-det.o
obj-$(CONFIG_SND) 			+= board-sirloin-headset-detect.o

obj-$(CONFIG_MMC_HSMMC_PE)		+= board-sirloin-3430-hsmmc.o

obj-$(CONFIG_FASTPATH) += board-sirloin-3430-wk.o

endif
