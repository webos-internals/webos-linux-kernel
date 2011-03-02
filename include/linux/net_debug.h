#ifndef __LINUX__NET_DEBUG_H
#define __LINUX__NET_DEBUG_H

enum {
	NDI_IP_OUTPUT = 0,
	NDI_IP_FINISH_OUTPUT,
	NDI_DEV_QUEUE_XMIT,
	NDI_PFIFO_FAST_ENQUEUE,
	NDI_QDISC_RESTART,
	NDI_PPP_START_XMIT,
	NDI_PPP_PUSH,
	NDI_MAX
};

struct net_debug_info_t {
	char *name;
	unsigned long last_time;
	void *last_skb;
	unsigned long count;
	int data1;
	int data2;
	int data3;
};

extern struct net_debug_info_t net_debug_info[];

#endif /* __LINUX__NET_DEBUG_H */
