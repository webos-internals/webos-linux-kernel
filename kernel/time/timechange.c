/** 
* @file timechange.c
* 
* Copyright (c) 2008 Palm, Inc. or its subsidiaries.
* 
* This software is distributed under the terms of the GNU General
* Public License ("GPL") as published by the Free Software Foundation,
* version 2 of the License.
*
* @brief Module that adds a timechange notifier list and broadcast timechange
*		 events out into userspace.
* 
* @author Ed Wei
* 
* @version  0.10  2008-08-19  <EW>  Initial creation
*/

#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#define TIMECHANGE_EVENT_NAME "timechange"

/** 
* @brief Notifier list for time change events.
* 
* @param  timechange_notifier_list 
*/
static ATOMIC_NOTIFIER_HEAD(timechange_notifier_list);

int timechange_notifier_register(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&timechange_notifier_list, nb);
}
EXPORT_SYMBOL(timechange_notifier_register);

int timechange_notifier_unregister(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&timechange_notifier_list, nb);
}
EXPORT_SYMBOL(timechange_notifier_unregister);

static struct work_struct timechange_work;

static struct platform_device timechange_event_device = {
	.name = TIMECHANGE_EVENT_NAME,
	.id = -1,
};

static struct platform_driver timechange_event_driver = {
	.driver = {
		.name		= TIMECHANGE_EVENT_NAME,
		.bus		= &platform_bus_type,
		.owner		= THIS_MODULE,
	},
};

/** 
* @brief The principle timechange callback which broadcasts a uevent
*		 that the time has been changed to userspace.
* 
* @param  work 
*/
static int timechange_callback(struct notifier_block *nb, unsigned long event, void *ptr)
{
	printk(KERN_INFO TIMECHANGE_EVENT_NAME ": time was changed.\n");

	kobject_uevent_env(&timechange_event_device.dev.kobj,
			KOBJ_CHANGE, NULL);
	return 0;
}

static struct notifier_block timechange_uevent_notifier = {
	.notifier_call = timechange_callback,
	.next = NULL,
	.priority = INT_MIN, /* Do this last. */
};

static void timechange_notify_list(struct work_struct *unused)
{
	atomic_notifier_call_chain(&timechange_notifier_list, 0, NULL);
}

/** 
* @brief Trigger notifications.
*/
void timechange_notify(void)
{
    schedule_work(&timechange_work);
}
EXPORT_SYMBOL(timechange_notify);

static int __init timechange_init(void)
{
	int ret = 0;

	INIT_WORK(&timechange_work, timechange_notify_list);

	// Register event device.
	if ((ret = platform_device_register(&timechange_event_device)))
		goto platform_device_register_fail;

	if ((ret = platform_driver_register(&timechange_event_driver)))
		goto driver_register_fail;

	// Register uevent notifier.
	timechange_notifier_register(&timechange_uevent_notifier);
	goto done;
driver_register_fail:
	platform_device_unregister(&timechange_event_device);
platform_device_register_fail:
done:
	return ret;
}

static void __exit timechange_exit(void)
{
	platform_driver_unregister(&timechange_event_driver);
	platform_device_unregister(&timechange_event_device);
}

module_init(timechange_init);
module_exit(timechange_exit);

MODULE_DESCRIPTION("Timechange event driver");
MODULE_AUTHOR("Palm, Inc.");
MODULE_LICENSE("GPL");
