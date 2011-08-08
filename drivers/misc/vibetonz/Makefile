obj-y := vibe.o

vibe-y := vibrator.o vtmdrv.o 
#vibe-y := vibrator.o 
vibe-$(CONFIG_OMAP_VIBRATOR) += omap_vibrator_plat.o
vibe-$(CONFIG_MSM_VIBRATOR) += msm_vibrator_plat.o
