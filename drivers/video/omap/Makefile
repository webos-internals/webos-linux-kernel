#
# Makefile for the new OMAP framebuffer device driver
#

ifeq ($(CONFIG_ARCH_OMAP1),y)
  obj-$(CONFIG_FB_OMAP) += omapfb.o
  objs-y := omapfb_main.o
  objs-$(CONFIG_ARCH_OMAP1)  += lcdc.o
  objs-$(CONFIG_ARCH_OMAP2) += dispc.o
  ifeq ($(CONFIG_FB_OMAP_LCDC_EXTERNAL),y)
    objs-$(CONFIG_ARCH_OMAP1) += sossi.o
    objs-$(CONFIG_ARCH_OMAP2) += rfbi.o
  endif

  objs-$(CONFIG_FB_OMAP_LCDC_HWA742) += hwa742.o
  objs-$(CONFIG_FB_OMAP_LCDC_BLIZZARD) += blizzard.o

  objs-$(CONFIG_MACH_AMS_DELTA) += lcd_ams_delta.o
  objs-$(CONFIG_MACH_OMAP_2430SDP) += omap2_disp_out.o
  objs-$(CONFIG_MACH_OMAP_H4) += lcd_h4.o
  objs-$(CONFIG_MACH_OMAP_H3) += lcd_h3.o
  objs-$(CONFIG_MACH_OMAP_H2) += lcd_h2.o
  objs-$(CONFIG_MACH_OMAP_PALMTE) += lcd_palmte.o
  objs-$(CONFIG_MACH_OMAP_PALMZ71) += lcd_palmz71.o
  objs-$(CONFIG_MACH_OMAP_PALMTT) += lcd_palmtt.o
  ifeq ($(CONFIG_MACH_OMAP_INNOVATOR),y)
    objs-$(CONFIG_ARCH_OMAP16XX) += lcd_inn1610.o
    objs-$(CONFIG_ARCH_OMAP15XX) += lcd_inn1510.o
  endif
  objs-$(CONFIG_MACH_SX1) += lcd_sx1.o
  objs-$(CONFIG_MACH_OMAP_OSK) += lcd_osk.o
  objs-$(CONFIG_MACH_OMAP_PERSEUS2) += lcd_p2.o
  objs-$(CONFIG_MACH_OMAP_APOLLON) += lcd_apollon.o

  objs-$(CONFIG_FB_OMAP_LCD_MIPID) += lcd_mipid.o
endif

ifeq ($(CONFIG_ARCH_OMAP2),y)
  obj-$(CONFIG_FB_OMAP)  += omap_disp_out.o
  obj-$(CONFIG_FB_OMAP)  += omap_fb.o
endif

ifeq ($(CONFIG_ARCH_OMAP3),y)
  ifeq ($(CONFIG_FB_OMAP),y)

    ifeq ($(CONFIG_MACH_SIRLOIN),y)
      obj-$(CONFIG_FB_OMAP_DISPLAY_CLASS) += lcd_class.o \
                                             lcd_panel.o
      obj-$(CONFIG_FB_OMAP_CONTROLLER) += ctrl_omap3.o
      obj-$(CONFIG_FB_OMAP) += omap_fb.o
      obj-$(CONFIG_FB_OMAP_PANEL_ACX567AKM) += panel_acx567akm.o
      obj-$(CONFIG_SPI_ACX567AKM_DBG) += panel_acx567akm_dbg.o
      obj-$(CONFIG_FB_OMAP_PANEL_FPD95220) += panel_fpd95220.o
      obj-$(CONFIG_FB_OMAP_BACKLIGHT_TPS61060) += bl_tps61060.o
      obj-$(CONFIG_FB_OMAP_BACKLIGHT_MAXIM8831) += bl_maxim8831.o
      obj-$(CONFIG_FB_OMAP_BACKLIGHT_EVT) += bl_evt.o
      obj-$(CONFIG_FB_OMAP_ON_VIDEO_LAYER) += omap_fbv.o
    endif

    ifeq ($(CONFIG_MACH_OMAP_3430SDP),y)
      obj-$(CONFIG_MACH_OMAP_3430SDP) += omap_disp_out.o
      obj-y += omap_fb.o
    endif
  endif
endif

omapfb-objs := $(objs-y)

