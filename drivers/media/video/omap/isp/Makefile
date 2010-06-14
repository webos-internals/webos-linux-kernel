# Makefile for OMAP3 ISP driver

ifdef CONFIG_ARCH_OMAP3410
obj-$(CONFIG_VIDEO_OMAP34XX_ISP) += \
	isp.o ispccdc.o ispmmu.o
else
obj-$(CONFIG_VIDEO_OMAP34XX_ISP) += \
	isp.o ispccdc.o isppreview.o ispresizer.o ispmmu.o isph3a.o isphist.o
obj-$(CONFIG_VIDEO_OMAP34XX_ISP_PREVIEWER) += \
	omap_previewer.o

obj-$(CONFIG_VIDEO_OMAP34XX_ISP_RESIZER) += \
	omap_resizer.o

endif

