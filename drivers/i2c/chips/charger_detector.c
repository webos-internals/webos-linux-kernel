/*
 * charger_detect.c
 *
 * Copyright (C) 2008-2009 Palm, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#ifdef CONFIG_USB_GADGET_EVENT
#include <linux/usb/gadget_event.h>
#endif

#include "charger_detector.h"

#define NAME	"charger_detector"

static int schedule_pending = 0; /* 1 if scheduled before init */

struct charger_detector_data {
	int vbus;
	int charger_mA;
	struct workqueue_struct *work_queue;
	struct delayed_work charger_detect_work;
	struct work_struct vbus_lost_work;
};

static struct charger_detector_data charger_detector_data = {
	.vbus = -1,
	.charger_mA = 0,
};

static struct charger_detector_data *the_data = NULL;

static void
charger_detect_work(struct work_struct *work)
{
	int vbus = 0;

	transceiver_vbus_presence(&vbus);
	if (the_data->vbus == vbus) {
		/* do nothing */
		printk(KERN_INFO "%s: vbus=%d (no change)\n", NAME, vbus);
		return;
	}
	if (vbus == 0) {
		printk(KERN_INFO "%s: vbus=%d\n", NAME, vbus);
#ifdef CONFIG_USB_GADGET_EVENT
		gadget_event_power_state_changed(G_EV_SOURCE_NONE, 0);
#endif
		return;
	}
	else if (vbus == 1) {
		unsigned long start, expires, now;
		int pullup = 0;

		/* check if pull-up is attached */
		start = jiffies;
		expires = start + msecs_to_jiffies(500); /* 500ms */

		for (;;) {
			now = jiffies;

			transceiver_is_pullup_attached(&pullup);
			if (pullup)
				break;
			if (time_after(now, expires))
				break;
			msleep(5);
		}
		if (!pullup) {  /* pull-up not detected */
			printk(KERN_INFO "%s: pullup not detected (> %ums)\n",
			       NAME, jiffies_to_msecs(now - start));
			return;
		}
		printk(KERN_INFO "%s: pullup detected (elapsed=%ums)\n",
		       NAME, jiffies_to_msecs(now - start));

		/* watch D+ and D- */
		start = jiffies;
		expires = start + msecs_to_jiffies(5000); /* 5000ms */

		for (;;) {
			int dplus = 1;
			int dminus = 0;

			now = jiffies;

			transceiver_single_ended_state(&dplus, &dminus);
			if (dplus && dminus) {
				/* this is a charger */
				printk(KERN_INFO "%s: charger(1000mA) detected (elapsed=%ums)\n", NAME, jiffies_to_msecs(now - start));
				the_data->charger_mA = 1000;
				break;
			}
			if (!dplus && !dminus) {
				/* this is a host */
				printk(KERN_INFO "%s: host detected (elapsed=%ums)\n", NAME, jiffies_to_msecs(now - start));
				break;
			}

			if (time_after(now, expires)) {
				if (dplus && !dminus) {
				/* REVISIT:
				   This charger doesn't conform to the USB
				   battery charging spec. Charge at 500 mA.
				   There shouldn't be danger of overcurrenting
				   a host because a host is required to
				   ground D+ and D-. */
					printk(KERN_INFO "%s: charger(500mA) detected (elapsed=%ums)\n", NAME, jiffies_to_msecs(now - start));
					the_data->charger_mA = 500;
				}
				break;
			}
			msleep(5);
		}
	}

#ifdef CONFIG_USB_GADGET_EVENT
	if (the_data->charger_mA > 0)
		gadget_event_power_state_changed(G_EV_SOURCE_CHARGER,
						 the_data->charger_mA);
#endif
}

static void
charger_vbus_lost_work(struct work_struct *work)
{
#ifdef CONFIG_USB_GADGET_EVENT
	gadget_event_power_state_changed(G_EV_SOURCE_NONE, 0);
#endif
}

int
charger_schedule_detection(void)
{
	if (the_data == NULL) {
		printk(KERN_INFO "%s: schudule detection (not ready)\n", NAME);
		schedule_pending = 1;
		return -1;
	}
	printk(KERN_INFO "%s: schudule detection\n", NAME);
	return queue_delayed_work(the_data->work_queue,
				  &the_data->charger_detect_work,
				  msecs_to_jiffies(500));
}
EXPORT_SYMBOL(charger_schedule_detection);

int
charger_cancel_detection(void)
{
	if (the_data == NULL) {
		printk(KERN_INFO "%s: cancel detection (not ready)\n", NAME);
		schedule_pending = 0;
		return -1;
	}
	printk(KERN_INFO "%s: cancel detection\n", NAME);
	return cancel_delayed_work_sync(&the_data->charger_detect_work);
}
EXPORT_SYMBOL(charger_cancel_detection);

int
charger_vbus_lost(void)
{
	if (the_data == NULL) {
		printk(KERN_INFO "%s: vbus lost (not ready)\n", NAME);
		schedule_pending = 0;
		return -1;
	}
	if (the_data->charger_mA == 0) {
		printk(KERN_INFO "%s: vbus lost (ignored)\n", NAME);
		return -1;
	}
	printk(KERN_INFO "%s: vbus lost\n", NAME);
	the_data->charger_mA = 0;
	return queue_work(the_data->work_queue, &the_data->vbus_lost_work);
}

static int __init init(void)
{
	the_data = &charger_detector_data;

	the_data->work_queue =
		create_singlethread_workqueue("charger_detector");
	if (!the_data->work_queue)
		return -1;

	INIT_DELAYED_WORK(&the_data->charger_detect_work, charger_detect_work);
	INIT_WORK(&the_data->vbus_lost_work, charger_vbus_lost_work);

	if (schedule_pending) {
		charger_schedule_detection();
		schedule_pending = 0;
	}

	return 0;
}
module_init(init);

static void __exit cleanup(void)
{
	if (the_data->work_queue)
		destroy_workqueue(the_data->work_queue);
	the_data = NULL;
}
module_exit(cleanup);
