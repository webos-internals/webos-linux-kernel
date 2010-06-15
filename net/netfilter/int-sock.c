//##################### This module is written to disconnect all the sockets using a particular interface as soon as the interface goes down  ###########################


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <linux/string.h>

#include <linux/socket.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/icmp.h>

#include <linux/netfilter.h>
#include <linux/netfilter_ipv4.h>

#include <linux/notifier.h>
#include <net/tcp.h>
#include <net/inet_hashtables.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <net/udp.h>
#include <linux/inetdevice.h>
#include <linux/gen_timer.h>


#define DEBUG(...)                           \
do {                                                         \
        if (intsock_debug_enabled)                \
                printk(__VA_ARGS__);                 \
} while (0)

/** 
* @brief Turn on debug messages.
*/
static uint32_t intsock_debug_enabled=0;
module_param_named(debug_level, intsock_debug_enabled, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug_level, "enable intsock debug.");
static int carrier_idle_time = 0;
static struct timer_list idle_sock_timer;
extern unsigned long current_sleep_length;

#define DEV_NAME "discsock"

extern struct inet_hashinfo tcp_hashinfo;

typedef struct worker
{
	struct work_struct work;
	spinlock_t poll_req_lock;
	struct list_head fin_head;
}worker_t;

static worker_t thr_ptr;

#define TCP_ESTABLISHED_TYPE	1
#define TCP_LISTENING_TYPE	2
#define UDP_TYPE		3	

typedef struct sock_entry
{
	struct list_head list;
	int type;
	int bucket;
	struct sock *sk;
}sock_entry_t;

/** 
* @brief Push a socket to be disconnected in the global queue used by the daemon
* 
*/

void push_sock_for_reset(struct sock *sk,int type,int bucket)
{
	struct sock_entry *fin = NULL;

	if((fin = (struct sock_entry *)kmalloc(sizeof(struct sock_entry), GFP_ATOMIC))==NULL)
	{
		return;
	}

	fin->sk = sk;
	fin->type=type;
	fin->bucket=bucket;
	spin_lock_bh(&thr_ptr.poll_req_lock);
	list_add_tail(&fin->list, &thr_ptr.fin_head);
	spin_unlock_bh(&thr_ptr.poll_req_lock);
	return;
}


/** 
* @brief Check if a TCP socket still exists in the established socket list
* 
*/
	
int check_tcp_established_socket(struct sock *this_sk, int bucket) 
{
	struct sock *sk;
	struct hlist_node *node;
	rwlock_t *lock = inet_ehash_lockp(&tcp_hashinfo, bucket);

	read_lock_bh(lock);
	sk_for_each(sk, node, &tcp_hashinfo.ehash[bucket].chain) {
		if (sk->sk_family != AF_INET) {
			continue;
		}
		if(sk==this_sk) {
			DEBUG("%s:Established socket %lx still in system list\n",
						__FUNCTION__,(unsigned long int)sk);
			read_unlock_bh(lock);
			return 1;
		}
	}
	read_unlock_bh(lock);
	return 0;
}

/** 
* @brief Check if a TCP socket still exists in the listening socket list
* 
*/
int check_tcp_listening_socket(struct sock *this_sk, int bucket) 
{

	struct sock *sk;
	struct hlist_node *node;

	sk_for_each(sk, node, &tcp_hashinfo.listening_hash[bucket]) {
		if (sk->sk_family != AF_INET) {
			continue;
		}
		if(sk==this_sk) {
			DEBUG("%s:Listening socket %lx still in system list\n",__FUNCTION__,
				(unsigned long int)sk);
			return 1;
		}
	}
	return 0;
}

/** 
* @brief Check if a UDP socket still exists in the UDP socket list
* 
*/

int check_udp_socket(struct sock *this_sk, int bucket) 
{
	struct sock *sk;
	struct hlist_node *node;
	struct hlist_head *udptable;

	udptable=udp_hash;
	read_lock(&udp_hash_lock);
	sk_for_each(sk, node, &(udptable[bucket])) {
		if (sk->sk_family != AF_INET)
			continue;

		if(sk==this_sk) {
			DEBUG("%s:UDP socket : %lx still in system list\n",__FUNCTION__,(unsigned long int)sk);
			read_unlock(&udp_hash_lock);
			return 1;
		}

	}

	read_unlock(&udp_hash_lock);
	return 0;
}


/** 
* @brief Check if the sock address is from one of the active interfaces, if not kill it by returning 1
* 
*/
__be32 ifaddr_list[16];

int check_addr_not_present(__be32 sock_addr)
{
	struct net_device *dev=NULL;
        struct in_device *in_dev=NULL;
	struct in_ifaddr **ifap = NULL;
	struct in_ifaddr *ifa = NULL;
	int ret = 1, i = 0;

	if(!sock_addr || sock_addr == 0x0100007F)
		return 0;

	if(ifaddr_list[0]) {
		while(ifaddr_list[i]) {
			if(sock_addr == ifaddr_list[i]) {
				DEBUG("%s: Addr : %lx sock addr : %lx\n",
						__FUNCTION__,(unsigned long int)ifaddr_list[i],
						(unsigned long int)sock_addr);
				return 0;
			}
			i++;
		}
		return 1;
	}

	read_lock(&dev_base_lock);
	for_each_netdev(&init_net, dev) {
		if(dev->flags & IFF_UP) {
			if ((in_dev = __in_dev_get_rtnl(dev)) != NULL) {
				for(ifap = &in_dev->ifa_list; (ifa = *ifap) != NULL;
						ifap = &ifa->ifa_next) {
					if(ifa) {
						DEBUG("%s:Interface %s  , addr : %lx sock addr : %lx\n",
								__FUNCTION__,ifa->ifa_label,(unsigned long int)ifa->ifa_local,
								(unsigned long int)sock_addr);
						ifaddr_list[i++] = ifa->ifa_local;		
						if(sock_addr == ifa->ifa_local) {
							ret = 0;
							break;
						}
					}

				}

			}
		}
	}

	read_unlock(&dev_base_lock);
	return ret;
}

/** 
* @brief Find idle tcp established sockets if check_idle_sock is set to 1, else find all the tcp 
*        sockets with source address not belonging to any active interface and push it in the
* 	 socket queue to destroy it.
*/

int find_tcp_established_sockets(int check_idle_sock, int offset) 
{
	int bucket;
	struct inet_sock *inet;
	__be32 src;
	struct timespec ts_curr, ts;
	int min_idle_time = 0, idle_time_remain = 0;

	for (bucket = 0; bucket < tcp_hashinfo.ehash_size; ++bucket) {
		struct sock *sk;
		struct hlist_node *node;
		rwlock_t *lock = inet_ehash_lockp(&tcp_hashinfo, bucket);

		read_lock_bh(lock);
		sk_for_each(sk, node, &tcp_hashinfo.ehash[bucket].chain) {
			if (sk->sk_family != AF_INET) {
				continue;
			}
			inet=inet_sk(sk);
			src = inet->rcv_saddr;

			if(check_idle_sock && (src != 0) && (src != 0x0100007F)) {
				ts=ktime_to_timespec(sk->sk_stamp);
				ts_curr=ktime_to_timespec(ktime_get_uptime());
				idle_time_remain=carrier_idle_time - (ts_curr.tv_sec + offset -ts.tv_sec);

				if(idle_time_remain <= 0)
					push_sock_for_reset(sk,TCP_ESTABLISHED_TYPE,bucket);
				else if(!min_idle_time || (min_idle_time > idle_time_remain))
					min_idle_time = idle_time_remain;
				DEBUG("%s: Socket %lx has idle time remaining as %d sec (sleep length : %d sec) ," 
					"min_idle_time : %d sec\n",__FUNCTION__,(unsigned long)sk,idle_time_remain,
					(int)current_sleep_length, min_idle_time);
			}
			else {
				if(check_addr_not_present(src)) { 
					DEBUG("%s:Established TCP socket found : %lx\n",__FUNCTION__,
							(unsigned long int)sk);
					push_sock_for_reset(sk,TCP_ESTABLISHED_TYPE,bucket);
				}
			}
		}
		read_unlock_bh(lock);
	}
	return min_idle_time;
}

/** 
* @brief Find idle tcp listening sockets if check_idle_sock is set to 1, else find all the tcp 
*        sockets with source address not belonging to any active interface and push it in the
*        socket queue to destroy it.
*/

int find_tcp_listening_sockets(int check_idle_sock, int offset) 
{
	int bucket;
	struct inet_sock *inet;
	__be32 src;
	struct timespec ts_curr, ts;
	int min_idle_time = 0, idle_time_remain = 0;

	for (bucket = 0; bucket < INET_LHTABLE_SIZE; ++bucket) {
		struct sock *sk;
		struct hlist_node *node;


		sk_for_each(sk, node, &tcp_hashinfo.listening_hash[bucket]) {
			if (sk->sk_family != AF_INET) {
				continue;
			}
			inet=inet_sk(sk);
			src = inet->rcv_saddr;

			if(check_idle_sock && (src != 0) && (src != 0x0100007F)) {
				ts=ktime_to_timespec(sk->sk_stamp);
				ts_curr=ktime_to_timespec(ktime_get_uptime());
				idle_time_remain=carrier_idle_time - (ts_curr.tv_sec + offset -ts.tv_sec);
				
				if(idle_time_remain <= 0)
					push_sock_for_reset(sk,TCP_LISTENING_TYPE,bucket);
				else if(!min_idle_time || (min_idle_time > idle_time_remain))
					min_idle_time = idle_time_remain;
				DEBUG("%s: Socket %lx has idle time remaining as %d sec (sleep length : %d sec)," 
					"min_idle_time : %d sec\n",__FUNCTION__,(unsigned long)sk,idle_time_remain,
					(int)current_sleep_length, min_idle_time);
			}
			else {

				if(check_addr_not_present(src)) {
					DEBUG("%s:Listening TCP socket found : %lx\n",__FUNCTION__,
							(unsigned long int)sk);
					push_sock_for_reset(sk,TCP_LISTENING_TYPE,bucket);
				}
			}
		}
	}
	return min_idle_time;
}

/** 
* @brief Find idle udp sockets if check_idle_sock is set to 1, else find all the udp 
*        sockets with source address not belonging to any active interface and push it in the
*        socket queue to destroy it.
*/


int find_udp_sockets(int check_idle_sock, int offset) 
{
	struct sock *sk;
	int bucket;
	struct hlist_head *udptable;
	struct inet_sock *inet;
	__be32 src;
	struct timespec ts_curr, ts;
	int min_idle_time = 0, idle_time_remain = 0;

	udptable=udp_hash;

	read_lock(&udp_hash_lock);
	for (bucket = 0; bucket < UDP_HTABLE_SIZE; ++bucket) {
		struct hlist_node *node;
		sk_for_each(sk, node, &(udptable[bucket])) {
			if (sk->sk_family != AF_INET)
				continue;
			inet=inet_sk(sk);
			src = inet->rcv_saddr;
			
			if(check_idle_sock && (src != 0) && (src != 0x0100007F)) {
				ts=ktime_to_timespec(sk->sk_stamp);
				ts_curr=ktime_to_timespec(ktime_get_uptime());
				idle_time_remain=carrier_idle_time - (ts_curr.tv_sec + offset -ts.tv_sec);
				
				if(idle_time_remain <= 0)
					push_sock_for_reset(sk,UDP_TYPE,bucket);
				else if(!min_idle_time || (min_idle_time > idle_time_remain))
					min_idle_time = idle_time_remain;
			}
			else {

				if(check_addr_not_present(src)) {
					DEBUG("%s:UDP socket found  : %lx\n",__FUNCTION__,
							(unsigned long int)sk);
					push_sock_for_reset(sk,UDP_TYPE,bucket);
				}
			}
		}
	}

	read_unlock(&udp_hash_lock);
	return min_idle_time;
}

/** 
* @brief Worker thread to destroy all sockets in the socket queue based on socket type.
*
*/

static void start_worker(struct work_struct *work)
{
	struct sock_entry *fin;
	struct sock *sk;


	spin_lock_bh(&thr_ptr.poll_req_lock);

	while(!list_empty(&thr_ptr.fin_head))
	{

		fin = list_entry(thr_ptr.fin_head.next, struct sock_entry, list);
		list_del_init(&fin->list);
		spin_unlock_bh(&thr_ptr.poll_req_lock);
		sk=fin->sk;
		lock_sock(sk);
		switch(fin->type) {
			case TCP_ESTABLISHED_TYPE:
				if(check_tcp_established_socket(sk,fin->bucket)) 
					tcp_disconnect(sk,0);
				break;
			case TCP_LISTENING_TYPE:
				if(check_tcp_listening_socket(sk,fin->bucket))
					tcp_disconnect(sk,0);
				break;

			case UDP_TYPE:
				if(check_udp_socket(sk,fin->bucket)) {
					sk->sk_err=ENONET;
					sk->sk_error_report(sk);
				}
				break;
		}
		release_sock(sk);
		kfree(fin);
		spin_lock_bh(&thr_ptr.poll_req_lock);


	}
	spin_unlock_bh(&thr_ptr.poll_req_lock);
	return;		
}


/** 
* @brief Initialize worker thread
*/

int init_worker(void)
{

	INIT_WORK(&thr_ptr.work,start_worker);
	INIT_LIST_HEAD(&thr_ptr.fin_head);
	spin_lock_init(&thr_ptr.poll_req_lock);
	return 0;
}


/** 
* @brief Exit worker thread
*/

int exit_worker(void)
{
	flush_scheduled_work();
	return 0;
}


/** 
* @brief Function called from dev_notifier_event to find and destroy all sockets
* belonging to an inactive interface
*/

void tear_down_interface_sockets(void) {
	memset(ifaddr_list,0,sizeof(ifaddr_list));
	find_tcp_established_sockets(0,0);
	find_tcp_listening_sockets(0,0);
	find_udp_sockets(0,0);
	schedule_work(&thr_ptr.work);
}

/** 
* @brief Function called from run_gen_timer when the timer for idle sockets expires
*/

void tear_down_idle_sockets(unsigned long arg) {
	find_tcp_established_sockets(1,current_sleep_length);
	find_tcp_listening_sockets(1,current_sleep_length);
	find_udp_sockets(1,current_sleep_length);
	schedule_work(&thr_ptr.work);
}

/** 
* @brief Function called from TCP fastpath code to add idle socket timer
*/

void check_idle_sockets(void) {
	int tmp_min_time = 0, min_time = 0;
	
	if(!carrier_idle_time)
		return;

	min_time = find_tcp_established_sockets(1,0);

	tmp_min_time = find_tcp_listening_sockets(1,0);
	if(tmp_min_time && min_time > tmp_min_time)
		min_time = tmp_min_time;
	
	tmp_min_time = find_udp_sockets(1,0);
	if(tmp_min_time && min_time > tmp_min_time)
		min_time = tmp_min_time;
	
	schedule_work(&thr_ptr.work);
	if(min_time > 0) {
		DEBUG("%s: Setting idle sock timer alarm for %d sec\n",__FUNCTION__,min_time);
		mod_gen_timer(&idle_sock_timer, jiffies + (min_time*HZ));
	}
}

EXPORT_SYMBOL(check_idle_sockets);


/** 
* @brief Notifier event to capture the event of interface going down, and in that case
*	 push all sockets on the global queue
*/
static int dev_notifier_event(struct notifier_block *this,
		unsigned long event, void *ptr)
{
	struct net_device *dev = ptr;
	switch (event) {
		case NETDEV_UNREGISTER:
			break;
		case NETDEV_REGISTER:
			break;
		case NETDEV_UP:
			break;
		case NETDEV_CHANGE:
		case NETDEV_DOWN:
			DEBUG("%s:Interface %s down/change , event : %s\n",__FUNCTION__,
				dev->name,(event==NETDEV_DOWN)?"NETDEV_DOWN":"NETDEV_CHANGE");
			if(!strcmp(dev->name,"lo"))
				break;
			tear_down_interface_sockets();			

			break;
		default:
			break;
	}
	return NOTIFY_DONE;
}



struct notifier_block dev_notifier = {
	dev_notifier_event,
	NULL,
	0
};

#define KILL_TCP_SOCK	1
#define KILL_UDP_SOCK	2
#define SET_CARRIER_IDLE_TIME	3

int kdev_ioctl(struct inode* inode, struct file* file, unsigned int cmd,
		unsigned long arg)
{

	struct sock *sk;
	if (!file || !inode)
	{
		return -1;
	}
	switch(cmd)
	{
		case KILL_TCP_SOCK:
			sk=(struct sock *)arg;
			tcp_disconnect(sk,0);
			break;
		case KILL_UDP_SOCK:
			sk=(struct sock *)arg;
			sk->sk_err=ENONET;
			sk->sk_error_report(sk);
			break;
		case SET_CARRIER_IDLE_TIME:
			carrier_idle_time = (int)arg;
			DEBUG("%s: Setting carrier idle time to %d secs\n",__func__,carrier_idle_time);
			break;
		default:
			break;
	}
	return 0;
}

struct file_operations kdev_fops =
{
	open: NULL,
        release: NULL,
        read: NULL,
        write: NULL,
        ioctl: kdev_ioctl
};

static int major_dev;
/* Initialisation routine */
static int __init interface_chk_init_module(void)
{

	memset(&thr_ptr,0,sizeof(worker_t));
	init_worker();

	register_netdevice_notifier(&dev_notifier);
	major_dev=register_chrdev(0,DEV_NAME,&kdev_fops);	// Registering as a char driver for ioctl call
	init_gen_timer(&idle_sock_timer,tear_down_idle_sockets,0);

	return 0;
}

/* Cleanup routine */
static void interface_chk_exit_module(void)
{
	exit_worker();
	unregister_netdevice_notifier(&dev_notifier);
	unregister_chrdev(major_dev,DEV_NAME);
}


module_init(interface_chk_init_module);
module_exit(interface_chk_exit_module);

MODULE_AUTHOR("Palm, Inc.");
MODULE_DESCRIPTION("Shuts down the sockets when the interface goes down or on an ioctl request");
MODULE_LICENSE("GPL");


