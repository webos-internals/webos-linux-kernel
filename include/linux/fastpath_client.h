#ifndef _FASTPATH_CLIENT_H
#define _FASTPATH_CLIENT_H

#define MAX_CLIENTNAME_LEN 32

typedef struct fastpath_client {
	const char *name;
        int (*prepare)(void);
        int (*fastwake)(void);
        struct list_head entry;
        unsigned long next_alarm;
}fastpath_client_t;


int register_fastpath_client(fastpath_client_t *client);
void unregister_fastpath_client(fastpath_client_t *client);


#endif
