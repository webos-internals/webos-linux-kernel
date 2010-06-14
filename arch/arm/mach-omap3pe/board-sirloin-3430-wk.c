/*
 * linux/arch/arm/mach-omap3pe/board-sirloin-3430-wk.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include <asm/arch/mux.h>
#include <asm/arch/control.h>

#include <asm/arch/pm.h>
#include <linux/wake_sources.h>

#include "board-sirloin-3430-wk.h"

/* Implementation of new wake source reporting API for sawmill tool.
 */
#ifdef CONFIG_WAKE_SOURCES
static struct wakeup_source sirloin_pm_wakeup_sources[] = {
	WAKEUP_SOURCE("UNKNOWN"),
	WAKEUP_SOURCE("MODEM"),
	WAKEUP_SOURCE("BT"),
	WAKEUP_SOURCE("WIFI"),
	WAKEUP_SOURCE("BUTTON"),
	WAKEUP_SOURCE("ALARM"),
	WAKEUP_SOURCE("GPIO"),
	WAKEUP_SOURCE("OTHER"),
};

static struct wakeup_source *wakeup_source_unknown;
static struct wakeup_source *wakeup_source_modem;
static struct wakeup_source *wakeup_source_bt;
static struct wakeup_source *wakeup_source_wifi;
static struct wakeup_source *wakeup_source_button;
static struct wakeup_source *wakeup_source_alarm;
static struct wakeup_source *wakeup_source_gpio;
static struct wakeup_source *wakeup_source_other;
#endif

/*************************************
 * Wakeup sources.
 ************************************/

struct wakeup_source_list {
	const char *name;
	const char *pin;
};

static struct wakeup_source_list omap3_wakeup_source[] = {
 /* bit      wake source name     muxpin name */
 /*  0 */ { "RTC_WAKE",          "AF26_SYS_NREQ" },
 /*  1 */ { "MODEM_WAKE_UART",   "AE1_3430_GPIO152" },
 /*  2 */ { "MODEM_WAKE_USB",    "AC1_3430_GPIO155" },
 /*  3 */ { "CORE_NAVI_WAKE",    "AE13_3430_GPIO17_CORE_NAVI" },
 /*  4 */ { "BT_WAKE",           "AB1_BT_HOST_WAKE" },
 /*  5 */ { "WIFI_WAKE",         "AF3_WL_IRQ" },
 /*  6 */ { "MODEM_UART_WAKE",   "Y8_3430_UART1_RX" },
 /*  7 */ { "BT_UART_WAKE",      "AD25_3430_UART2_RX" },
 /*  8 */ { "KEYPAD",            "AE10_3430_GPIO13_KEY_INT" },
 /*  9 */ { "KEY_PTT",           "AG7_3430_GPIO26_KEY_PTT" },
 /* 10 */ { "SLIDER",            "AH7_3430_GPIO27_SLIDER_OPEN" },
 /* 11 */ { "RINGER_SWITCH",     "AG8_3430_GPIO28_RING" },
 /* 12 */ { "POWER_BUTTON",      "AH8_3430_GPIO29_PWR_WIFI_KEY" },
 /* 13 */ { "HEADSET_INSERT",    "B26_3430_GPIO111" },
 /* 14 */ { "HEADSET_BUTTON",    "H27_3430_GPIO087" },
};

static u32 omap3_wakeup_source_info;

/** 
 * @brief This must be called from prcm_pwr.c before the wakeup
 *        source registers get clobbered.
 */
void omap3_wakeup_sources_save(void)
{
	u16 mask = 0x8000;
	u16 addr, val;
	int i;
#ifdef CONFIG_WAKE_SOURCES
	int have_valid_reason = 0;
#endif

	omap3_wakeup_source_info = 0;

	for (i = 0; i < ARRAY_SIZE(omap3_wakeup_source); i++) {
		addr = omap_get_mux_reg(omap3_wakeup_source[i].pin);

		if (addr) {
			val = omap_ctrl_readw(addr);

			if (val & mask) {
				omap3_wakeup_source_info |= (1 << i);
			}
		}
	}

#ifdef CONFIG_WAKE_SOURCES
	for (i = 0; i < ARRAY_SIZE(omap3_wakeup_source); i++) {
		if (omap3_wakeup_source_info & (1<<i)) {
			struct wakeup_source *source;

			/* Create wake-up source events for the new wake source
			 * reporting API. Note that the index of the wake
			 * source corresponds to the type of event. If the
			 * wake_source_list table is being changed then also
			 * the sirloin_pm_wakeup_sources table need to be
			 * changed.
			 */
			switch (i) {
			case 0:  source = wakeup_source_alarm; break;
				 
			case 1:
			case 2:
			case 6:  source = wakeup_source_modem; break;

			case 3:
			case 8:
			case 9:
			case 12:
			case 14: source = wakeup_source_button; break;

			case 4:
			case 7:  source = wakeup_source_bt; break;

			case 5:  source = wakeup_source_wifi; break;

			case 10:
			case 11:
			case 13: source = wakeup_source_gpio; break;

			default: source = wakeup_source_unknown; break;
			}
			wakeup_event(source, omap3_wakeup_source[i].name);
			have_valid_reason++;
		}
	}

	/* Still no valid wakeup reason found? */
	if (!have_valid_reason)
		wakeup_event(wakeup_source_unknown, NULL);
#endif
}

static ssize_t omap3_wakeup_sources_show(struct kset *subsys, char *buf)
{
	int len = 0;
	int i;
	for (i = 0; i < ARRAY_SIZE(omap3_wakeup_source); i++) {
		
		struct wakeup_source_list *source = &omap3_wakeup_source[i];
		if (omap3_wakeup_source_info & (1 << i)) {
			len += sprintf(buf + len, "%s ", source->name);
		}
	}

	len += sprintf(buf + len, "\n");
	return len;
}

/* This function returns TRUE if RTC was the only wakeup source in the system.
 * This information will be used to determine whether or not fastpath can be
 * taken.
 *
 * Note that we ignore certain wakeup sources as there is a bug in the OMAP
 * silicon that causes those sources to "randomly" appear.
 */
int omap3_wakeup_is_rtc_only(void)
{
	u32 mask = omap3_wakeup_source_info & 
		~(WAKEUP_SOURCE_RINGER_SWITCH | WAKEUP_SOURCE_POWER_BUTTON);

	return mask == WAKEUP_SOURCE_RTC;
}
EXPORT_SYMBOL(omap3_wakeup_is_rtc_only);

int omap3_wakeup_sources_get(void)
{
	return omap3_wakeup_source_info;
}
EXPORT_SYMBOL(omap3_wakeup_sources_get);

void omap3_wakeup_sources_clear(void)
{
	omap3_wakeup_source_info = 0;
}

/**
 * Init
 */

static struct subsys_attribute wakeup_sources = {
	.attr = {
		.name = __stringify(wakeup_sources),
		.mode = 0644,
	},
	.show  = omap3_wakeup_sources_show,
	.store = NULL,
};

static int __init wakeup_source_info_init(void)
{
	int retval;

#ifdef CONFIG_WAKE_SOURCES
	init_wakeup_sources(sirloin_pm_wakeup_sources,
			sizeof(sirloin_pm_wakeup_sources) / sizeof(struct wakeup_source));

	wakeup_source_unknown = get_wakeup_source("UNKNOWN");
	wakeup_source_modem   = get_wakeup_source("MODEM");
	wakeup_source_bt      = get_wakeup_source("BT");
	wakeup_source_wifi    = get_wakeup_source("WIFI");
	wakeup_source_button  = get_wakeup_source("BUTTON");
	wakeup_source_alarm   = get_wakeup_source("ALARM");
	wakeup_source_gpio    = get_wakeup_source("GPIO");
	wakeup_source_other   = get_wakeup_source("OTHER");
#endif

	omap3_wakeup_source_info = 0;

	retval = subsys_create_file(&power_subsys, &wakeup_sources);
	if (retval) {
		printk(KERN_ERR
			"ERROR creating sysfs entry for 'wakeup_source': %d\n", retval);
	}

	return retval;
}

module_init(wakeup_source_info_init);
