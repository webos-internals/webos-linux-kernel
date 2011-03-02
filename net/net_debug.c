#ifdef CONFIG_PROC_FS

#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/ptrace.h>

#include <linux/net_debug.h>

/*
 * net_debug_info
 */
struct net_debug_info_t net_debug_info[NDI_MAX];

static void
init_ndi_names(void)
{
	net_debug_info[NDI_IP_OUTPUT].name = "ip_output";
	net_debug_info[NDI_IP_FINISH_OUTPUT].name = "ip_finish_output";
	net_debug_info[NDI_DEV_QUEUE_XMIT].name = "dev_queue_xmit";
	net_debug_info[NDI_PFIFO_FAST_ENQUEUE].name = "pfifo_fast_enqueue";
	net_debug_info[NDI_QDISC_RESTART].name = "qdisc_restart";
	net_debug_info[NDI_PPP_START_XMIT].name = "ppp_start_xmit";
	net_debug_info[NDI_PPP_PUSH].name = "ppp_push";
}

static ssize_t
read_net_debug_info(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	char lbuf[1024];
	ssize_t lcnt = 0;
	int i;
	struct net_debug_info_t info[NDI_MAX];

	memcpy(info, net_debug_info, sizeof(info));

	for (i = 0; i < NDI_MAX; i++) {
		lcnt += snprintf(lbuf+lcnt, sizeof(lbuf)-lcnt,
				 "%-18s %10lu %p %10lu %08x %08x %08x\n",
				 info[i].name,
				 info[i].last_time,
				 info[i].last_skb,
				 info[i].count,
				 info[i].data1,
				 info[i].data2,
				 info[i].data3
			);
	}

	if (p >= lcnt)
		return 0;
	if (count > lcnt - p)
		count = lcnt - p;

	if (copy_to_user(buf, lbuf+p, count))
		return -EFAULT;

	*ppos += count;
	return count;
}

static const struct file_operations proc_net_debug_info_operations = {
	.read		= read_net_debug_info,
};

static int __init net_debug_info_init(void)
{
	struct proc_dir_entry *entry;

	if (!(entry = create_proc_entry("net_debug_info", S_IRUGO, NULL)))
		return 0;
	entry->proc_fops = &proc_net_debug_info_operations;

	init_ndi_names();

	return 0;
}

/*
 * init
 */
static int __init net_debug_init(void)
{
	net_debug_info_init();

	return 0;
}
module_init(net_debug_init);
#endif /* CONFIG_PROC_FS */
