obj-y += io.o irq.o timer.o dma.o memory.o
obj-y += msm_adm.o msm_hsuart.o
obj-y += proc_comm.o mux.o
obj-y += vreg.o mpp.o
obj-y += clock.o clock-pcom.o
obj-y += gpio.o generic_gpio.o
obj-y += nand_partitions.o
obj-y += remote_spinlock.o modem_notifier.o
obj-y += socinfo.o
obj-$(CONFIG_CPU_FREQ) += cpufreq.o

obj-$(CONFIG_ARCH_MSM) += acpuclock.o internal_power_rail.o
obj-$(CONFIG_ARCH_QSD) += acpuclock-8x50.o sirc.o
obj-$(CONFIG_ARCH_QSD) += pmic.o

# Common code for board init
obj-y += common.o

obj-$(CONFIG_CPU_V6) += idle-v6.o
obj-$(CONFIG_CPU_V7) += idle-v7.o

obj-$(CONFIG_MSM_FIQ_SUPPORT) += fiq_glue.o
obj-$(CONFIG_MACH_CHUCK) += board-chuck.o board-chuck-headset-detect.o
obj-$(CONFIG_MSM_SMD) += smd.o smd_tty.o smd_qmi.o smem_log.o
obj-$(CONFIG_MSM_SMD) += smd_ctl2.o smd_loopback.o smd_nmea.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_dog_keepalive.o
obj-$(CONFIG_MSM_RPCSERVERS) += rpc_server_time_remote.o
obj-$(CONFIG_MSM_ADSP) += qdsp5/
obj-$(CONFIG_MSM_HW3D) += hw3d.o
obj-$(CONFIG_MSM_PERF) += perf.o
ifdef CONFIG_MSM_N_WAY_SMSM
	obj-$(CONFIG_PM) += board-chuck-pm2.o
else
	obj-$(CONFIG_PM) += pm.o
endif

obj-$(CONFIG_MSM_PWM) += pwm.o
obj-$(CONFIG_MSM_UARTDM) += msm_uart_dm.o
obj-$(CONFIG_PALM_NDUID) += uid.o
obj-$(CONFIG_MSM_POWER_DEBUG) += power_debug.o
obj-$(CONFIG_CPU_FREQ) += cpufreq.o



