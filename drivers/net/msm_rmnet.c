/* linux/drivers/net/msm_rmnet.c
 *
 * Virtual Ethernet Interface for MSM7K Networking
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>

#include <asm/arch/msm_smd.h>

/* XXX should come from smd headers */
#define SMD_PORT_ETHER0 11
#define SMD_POLL_MILLISECS 1


struct rmnet_private
{
	smd_channel_t *ch;
	struct net_device_stats stats;
	const char *chname;
	struct sk_buff *skb;
	struct timer_list smd_poll_timer;
};

static int count_this_packet(void *_hdr, int len)
{
	struct ethhdr *hdr = _hdr;

	if (len < ETH_HLEN)
		return 1;

	if (hdr->h_proto == htons(ETH_P_ARP))
		return 0;

	return 1;
}

/* Called in soft-irq context */
static void smd_net_data_handler(unsigned long arg)
{
	struct net_device *dev = (struct net_device *) arg;
	struct rmnet_private *p = netdev_priv(dev);
	struct sk_buff *skb;
	void *ptr = 0;
	int sz;

	for (;;) {
		sz = smd_cur_packet_size(p->ch);
		if (sz == 0) break;
		if (smd_read_avail(p->ch) < sz) break;

		if (sz > 1514) {
			pr_err("rmnet_recv() discarding %d len\n", sz);
			ptr = 0;
		} else {
			skb = dev_alloc_skb(sz + NET_IP_ALIGN);
			if (skb == NULL) {
				pr_err("rmnet_recv() cannot allocate skb\n");
			} else {
				skb->dev = dev;
				skb_reserve(skb, NET_IP_ALIGN);
				ptr = skb_put(skb, sz);
				if (smd_read(p->ch, ptr, sz) != sz) {
					pr_err("rmnet_recv() smd lied "
					       "about avail?!");
					ptr = 0;
					dev_kfree_skb_irq(skb);
				} else {
					skb->protocol = eth_type_trans(skb, dev);
					if (count_this_packet(ptr, skb->len)) {
						p->stats.rx_lasttime =  jiffies;
						p->stats.rx_packets++;
						p->stats.rx_bytes += skb->len;
					}
					netif_rx(skb);
				}
				continue;
			}
		}
		if (smd_read(p->ch, ptr, sz) != sz)
			pr_err("rmnet_recv() smd lied about avail?!");
	}
}

static DECLARE_TASKLET(smd_net_data_tasklet, smd_net_data_handler, 0);

static void smd_net_notify(void *_dev, unsigned event)
{
	if (event != SMD_EVENT_DATA)
		return;

	smd_net_data_tasklet.data = (unsigned long) _dev;

	tasklet_schedule(&smd_net_data_tasklet);
}

static int rmnet_open(struct net_device *dev)
{
	int r;
	struct rmnet_private *p = netdev_priv(dev);

	pr_info("rmnet_open(), channel name: %s\n",p->chname);
	if (!p->ch) {
		r = smd_open(p->chname, &p->ch, dev, smd_net_notify);

		if (r < 0)
			return -ENODEV;
	}

	netif_start_queue(dev);
	return 0;
}

static int rmnet_stop(struct net_device *dev)
{
	pr_info("rmnet_stop()\n");
	netif_stop_queue(dev);
	return 0;
}


static void rmnet_poll_smd_write(unsigned long param)
{
	struct net_device * dev = (struct net_device *)param;
	struct rmnet_private *p = netdev_priv(dev);
	smd_channel_t *ch = p->ch;
	int smd_ret;

	/* this should never happen */
	if (p == 0 || (p->skb) == 0 || (p->skb->len) == 0) {
		pr_err("fatal: rmnet rmnet_poll_smd_write rcvd skb length 0");
		return;
	}

	if (smd_write_avail(ch) >= p->skb->len) {
		smd_ret = smd_write(ch, p->skb->data, p->skb->len);
		if (smd_ret != p->skb->len) {
			pr_err("fatal: smd_write returned error %d", smd_ret);
		} else {
			if (count_this_packet(p->skb->data, p->skb->len)) {
				p->stats.tx_lasttime =  jiffies;
				p->stats.tx_packets++;
				p->stats.tx_bytes += smd_ret;
			}
		}

		/* data xmited, safe to release skb */
		dev_kfree_skb_irq(p->skb);
		p->skb = 0;
		netif_wake_queue((struct net_device *)dev);
	} else {
		init_timer(&p->smd_poll_timer);
		p->smd_poll_timer.expires = jiffies +
			((SMD_POLL_MILLISECS*HZ)/1000);
		p->smd_poll_timer.function = rmnet_poll_smd_write;
		p->smd_poll_timer.data = param;
		add_timer(&p->smd_poll_timer);
	}
}

static int rmnet_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	smd_channel_t *ch = p->ch;
	int smd_ret = 0;

	if (netif_queue_stopped(dev)) {
		pr_err("fatal: rmnet_xmit called when netif_queue is stopped");
		return 0;
	}

	if (smd_write_avail(ch) < skb->len) {
		rmnet_stop(dev);
		/* schedule a function to poll at exponential interval */
		init_timer(&p->smd_poll_timer);
		p->smd_poll_timer.expires = jiffies
			+ ((SMD_POLL_MILLISECS*HZ)/1000);
		p->smd_poll_timer.function = rmnet_poll_smd_write;
		if (p->skb)
			pr_err("fatal: p->skb was non-zero when"
				"we tried to scheduled timer");
		p->skb = skb;
		p->smd_poll_timer.data = (unsigned long)dev;
		add_timer(&p->smd_poll_timer);
	} else {
		smd_ret = smd_write(ch, skb->data, skb->len);
		if (smd_ret != skb->len) {
			pr_err("fatal: smd_write returned error %d", smd_ret);
		} else {
			if (count_this_packet(skb->data, skb->len)) {
				p->stats.tx_lasttime =  jiffies;
				p->stats.tx_packets++;
				p->stats.tx_bytes += smd_ret;
			}
		}
		/* data xmited, safe to release skb */
		dev_kfree_skb_irq(skb);
	}
	return 0;
}

static struct net_device_stats *rmnet_get_stats(struct net_device *dev)
{
	struct rmnet_private *p = netdev_priv(dev);
	return &p->stats;
}

static void rmnet_set_multicast_list(struct net_device *dev)
{
}

static void rmnet_tx_timeout(struct net_device *dev)
{
	pr_info("rmnet_tx_timeout()\n");
}

static void __init rmnet_setup(struct net_device *dev)
{
	dev->open = rmnet_open;
	dev->stop = rmnet_stop;
	dev->hard_start_xmit = rmnet_xmit;
	dev->get_stats = rmnet_get_stats;
	dev->set_multicast_list = rmnet_set_multicast_list;
	dev->tx_timeout = rmnet_tx_timeout;

	dev->watchdog_timeo = 20; /* ??? */

	ether_setup(dev);

	dev->change_mtu = 0; /* ??? */

	random_ether_addr(dev->dev_addr);
}


static const char *ch_name[3] = {
	"DATA5",
	"DATA6",
	"DATA7",
};

static int __init rmnet_init(void)
{
	int ret;
	struct net_device *dev;
	struct rmnet_private *p;
	unsigned n;

	for (n = 0; n < 3; n++) {
		dev = alloc_netdev(sizeof(struct rmnet_private),
				   "rmnet%d", rmnet_setup);

		if (!dev)
			return -ENOMEM;

		p = netdev_priv(dev);
		p->skb = 0;
		p->chname = ch_name[n];
		p->stats.rx_lasttime = 0;
		p->stats.tx_lasttime = 0;

		ret = register_netdev(dev);
		if (ret) {
			free_netdev(dev);
			return ret;
		}
	}
	return 0;
}


module_init(rmnet_init);
