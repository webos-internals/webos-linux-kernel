#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <asm/arch/pm.h>

#if defined(CONFIG_PM) && defined(CONFIG_SYSFS)

/******************************************************************************
 *
 * enable off
 *
 ******************************************************************************/

int enable_off = 1;

static ssize_t omap_pm_off_show(struct kset *subsys, char *buf)
{
	return sprintf(buf, "%hu\n", enable_off);
}

static ssize_t omap_pm_off_store(struct kset *subsys,
				const char *buf, size_t n)
{
	unsigned short value;
	if (sscanf(buf, "%hu", &value) != 1 ||
		((value != 0) && (value != 1))) {
		printk(KERN_ERR "off_enable: Invalid value\n");
		return -EINVAL;
	}
	enable_off = value;
	return n;
}


static struct subsys_attribute off_enable = {
	.attr = {
		.name = __stringify(enable_mpucoreoff),
		.mode = 0644,
	},
	.show  = omap_pm_off_show,
	.store = omap_pm_off_store,
};

int __init omap3_pm_sysfs_init(void)
{
	int rc;

	rc = subsys_create_file(&power_subsys, &off_enable);
	if (rc)
		printk(KERN_ERR "subsys_create_file failed for off: %d\n", rc);
	return rc;
}

#endif /* #if defined(CONFIG_PM) && defined(CONFIG_SYSFS) */
