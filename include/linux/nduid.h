#ifndef __ASM_ARCH_NDUID_H__
#define __ASM_ARCH_NDUID_H__

struct nduid_config {
	char *dev_name;
	unsigned int (*get_device_salt)(void);
	int (*get_cpu_id)(char *id, unsigned int maxlen);
};

#endif
