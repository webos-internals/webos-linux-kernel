/*
 * linux/include/asm-arm/arch-omap/resource.h
 * Structure definitions for shared resource framework on OMAP34XX
 *
 * Copyright (C) 2006-2007 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 */

#ifndef __ARCH_ARM_OMAP_RESOURCE_H
#define __ARCH_ARM_OMAP_RESOURCE_H

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <asm/semaphore.h>
#include <linux/notifier.h>


/* Flags to denote Pool usage */
#define RES_UNUSED		0x0
#define RES_USED		0x1

/* Need to see if these static defines can be optimized */
#define MAX_USERS		100
#define	MAX_HANDLES		250

/* Overall max levels for the resources */
/* Constraints has 6 levels, future addition of resources with more levels
   would change this value */
#define MAX_LEVEL		0xE
#define DEFAULT_LEVEL		0x0

/* Type of resources */
#define RES_POWER_DOMAIN 	0x1 /* Power domains */
#define RES_LOGICAL		0x2 /* Logical resources */
#define RES_DPLL		0x3 /* DPLL resources */
#define RES_MEMORY_LOGIC	0x4 /* Memory resources */
#define RES_LATENCY_CO		0x5 /* Latency constraint */
#define	RES_FREQ_CO		0x6 /* Frequency constraint */
#define RES_OPP_CO		0x7 /* OPP constraint */
#define RES_CLOCK_RAMPUP_CO	0x8 /* Clock ramp up constraint */
#define RES_T2_POWER 		0x9 /* Triton power resources */

/* shared_resource levels */
#define DPLL_RES_MAXLEVEL               0x2
#define DPLL_RES_CORE_MAXLEVEL          0x6

/* Dpll resources */
#define DPLL_NO_AUTOIDLE                0x0
#define DPLL_AUTOIDLE                   0x1
#define DPLL_AUTOIDLE_BYPASS            0x5

/* memory and logic states */
#define MEMORY_OFF		 	0x0
#define MEMORY_RET	 		0x1
#define MEMORY_ON	  		0x3
#define MEMORY_MAXLEVEL_DOMAINRET	0x2
#define MEMORY_MAXLEVEL_DOMAINON	0x4

#define LOGIC_OFF	 		0x0
#define LOGIC_RET 			0x1
#define LOGIC_MAXLEVEL			0x2


/* Power domain levels */
#define POWER_DOMAIN_OFF 	0x0
#define POWER_DOMAIN_RET 	0x1
#define POWER_DOMAIN_ON  	0x3
#define POWER_DOMAIN_MAXLEVEL	0x4

/* Logical resource levels */
#define LOGICAL_UNUSED		0x0
#define LOGICAL_USED		0x1
#define	LOGICAL_MAXLEVEL	0x2

/* DPLL resource levels */
#define DPLL_LOW_POWER_STOP	0x0
#define DPLL_LOW_POWER_BYPASS	0x1
#define DPLL_FAST_RELOCK_BYPASS 0x2
#define DPLL_LOCKED		0x3
#define DPLL_MAXLEVEL		0x4

/* Constraints levels */
#define CO_UNUSED		0x0

/* Latency constraint levels */
#define CO_LATENCY_WFI			0x1
#define CO_LATENCY_MPURET_COREON	0x2
#define CO_LATENCY_MPUOFF_COREON	0x3
#define CO_LATENCY_MPUOFF_CORERET	0x4

/* Power domain latency constraint levels */
#define CO_LATENCY_ON			0x1
#define CO_LATENCY_RET			0x2

/* VDD1 OPP constraint levels */
#define CO_VDD1_OPP1		0x1
#define CO_VDD1_OPP2		0x2
#define CO_VDD1_OPP3		0x3
#define CO_VDD1_OPP4		0x4
#define CO_VDD1_OPP5		0x5

/* VDD2 OPP constraint levels */
#define CO_VDD2_OPP1		0x1
#define CO_VDD2_OPP2		0x2
#define CO_VDD2_OPP3		0x3

/* T2 Power Resources */

/* VAUX1 LDO voltage levels */
#define T2_VAUX1_OFF	0x0
#define T2_VAUX1_1V50  	0x1
#define T2_VAUX1_1V80  	0x2
#define T2_VAUX1_2V50  	0x3
#define T2_VAUX1_2V80  	0x4
#define T2_VAUX1_3V00  	0x5
#define T2_VAUX1_MAX   	0x6

/* VAUX2 LDO voltage levels */
#define T2_VAUX2_OFF   	0x0
#define T2_VAUX2_0V70  	0x1
#define T2_VAUX2_1V00  	0x2
#define T2_VAUX2_1V20  	0x3
#define T2_VAUX2_1V30  	0x4
#define T2_VAUX2_1V50  	0x5
#define T2_VAUX2_1V80  	0x6
#define T2_VAUX2_1V85  	0x7
#define T2_VAUX2_2V50	0x8
#define T2_VAUX2_2V60	0x9
#define T2_VAUX2_2V80	0xA
#define T2_VAUX2_2V85 	0xB
#define T2_VAUX2_3V00  	0xC
#define T2_VAUX2_3V15  	0xD
#define T2_VAUX2_MAX   	0xE

/*VAUX3 LDO voltage levels */
#define T2_VAUX3_OFF 	0x0
#define T2_VAUX3_1V50	0x1
#define T2_VAUX3_1V80	0x2
#define T2_VAUX3_2V50	0x3
#define T2_VAUX3_2V80	0x4
#define T2_VAUX3_3V00	0x5
#define T2_VAUX3_MAX 	0x6

/* VAUX4 LDO voltage levels */
#define T2_VAUX4_OFF 	0x0
#define T2_VAUX4_0V70	0x1
#define T2_VAUX4_1V00	0x2
#define T2_VAUX4_1V20	0x3
#define T2_VAUX4_1V30	0x4
#define T2_VAUX4_1V50	0x5
#define T2_VAUX4_1V80	0x6
#define T2_VAUX4_1V85	0x7
#define T2_VAUX4_2V50	0x8
#define T2_VAUX4_2V60	0x9
#define T2_VAUX4_2V80	0xA
#define T2_VAUX4_2V85	0xB
#define T2_VAUX4_3V00	0xC
#define T2_VAUX4_3V15	0xD
#define T2_VAUX4_MAX 	0xE

/* VMMC1 LDO voltage l evels */
#define T2_VMMC1_OFF    0x0
#define T2_VMMC1_1V85   0x1
#define T2_VMMC1_2V85   0x2
#define T2_VMMC1_3V00   0x3
#define T2_VMMC1_3V15   0x4
#define T2_VMMC1_MAX    0x5

/* VMMC2 LDO voltage l evels */
#define T2_VMMC2_OFF    0x0
#define T2_VMMC2_0V70   0x1
#define T2_VMMC2_1V00   0x2
#define T2_VMMC2_1V20   0x3
#define T2_VMMC2_1V30   0x4
#define T2_VMMC2_1V50   0x5
#define T2_VMMC2_1V80   0x6
#define T2_VMMC2_1V85   0x7
#define T2_VMMC2_2V50   0x8
#define T2_VMMC2_2V60   0x9
#define T2_VMMC2_2V80   0xA
#define T2_VMMC2_2V85   0xB
#define T2_VMMC2_3V00   0xC
#define T2_VMMC2_3V15   0xD
#define T2_VMMC2_MAX    0xE

/* VPLL2 LDP Voltage levels */
#define T2_VPLL2_OFF 	0x0
#define T2_VPLL2_0V70	0x1
#define T2_VPLL2_1V00	0x2
#define T2_VPLL2_1V20	0x3
#define T2_VPLL2_1V30	0x4
#define T2_VPLL2_1V50	0x5
#define T2_VPLL2_1V80	0x6
#define T2_VPLL2_1V85	0x7
#define T2_VPLL2_2V50	0x8
#define T2_VPLL2_2V60	0x9
#define T2_VPLL2_2V80	0xA
#define T2_VPLL2_2V85	0xB
#define T2_VPLL2_3V00	0xC
#define T2_VPLL2_3V15	0xD
#define T2_VPLL2_MAX 	0xE

/* VSIM LDO voltage levels */
#define T2_VSIM_OFF 	0x0
#define T2_VSIM_1V00	0x1
#define T2_VSIM_1V20	0x2
#define T2_VSIM_1V30	0x3
#define T2_VSIM_1V80	0x4
#define T2_VSIM_2V80	0x5
#define T2_VSIM_3V00	0x6
#define T2_VSIM_MAX 	0x7

/* VDAC LDO voltage levels */
#define T2_VDAC_OFF 	0x0
#define T2_VDAC_1V20	0x1
#define T2_VDAC_1V30	0x2
#define T2_VDAC_1V80	0x3
#define T2_VDAC_MAX 	0x4

/* VINTANA2 LDO voltage levels */
#define T2_VINTANA2_OFF  0x0
#define T2_VINTANA2_2V50 0x1
#define T2_VINTANA2_2V75 0x2
#define T2_VINTANA2_MAX  0x3

/* VIO LDO voltage levels */
#define T2_VIO_OFF 	0x0
#define T2_VIO_1V80	0x1
#define T2_VIO_1V85	0x2
#define T2_VIO_MAX	0x3

/* VBUS1V5 LDO levels */
#define T2_VUSB1V5_OFF  0x0
#define T2_VUSB1V5_1V5  0x1
#define T2_VUSB1V5_MAX  0x2

/* VBUS1V8 LDO levels */
#define T2_VUSB1V8_OFF  0x0
#define T2_VUSB1V8_1V8  0x1
#define T2_VUSB1V8_MAX  0x2

/* VBUS3V1 LDO levels */
#define T2_VUSB3V1_OFF  0x0
#define T2_VUSB3V1_3V1  0x1
#define T2_VUSB3V1_MAX  0x2

/* Frequencies and Voltages for OPPs */
#define MIN_VDD1_OPP 	CO_VDD1_OPP1
#define MAX_VDD1_OPP 	CO_VDD1_OPP5
#define MIN_VDD2_OPP 	CO_VDD2_OPP2
#define MAX_VDD2_OPP 	CO_VDD2_OPP3

/* 
 * For legacy code these defines should be the same as the ones in caps 
 */
#define min_vdd1_opp	MIN_VDD1_OPP
#define max_vdd1_opp	MAX_VDD1_OPP
#define min_vdd2_opp	MIN_VDD2_OPP
#define max_vdd2_opp	MAX_VDD2_OPP

/* Maximum number of constrain levels */
#define CO_MAXLEVEL	  (MAX_VDD1_OPP + 1)
#define CO_VDD1_MAXLEVEL  (MAX_VDD1_OPP + 1)
#define CO_VDD2_MAXLEVEL  (MAX_VDD2_OPP + 1)


/* Structure returned to the framework user as handle */
struct res_handle {
	struct 	shared_resource *res;
	const char *usr_name;
	struct notifier_block *nb_internal_pre[MAX_LEVEL];
	struct notifier_block *nb_internal_post[MAX_LEVEL];
	short	res_index;
	short	usr_index;
};

/* constraints API public structures */

/**
 * struct constraint_data - constraint description
 * @type: constraint type
 * @data: constraint data. internals vary depending on the type
 *  
 */
struct constraint_id {
	int   type;
	void *data;
};

typedef struct constraint_handle res_handle;
	
/* One instance of this structure exists for each shared resource
   which is modeled */
struct res_handle_node {
	struct res_handle *handle;
	struct list_head node;
};


struct shared_resource {
	const char 	*name;
	unsigned long 	prcm_id;
	unsigned long 	res_type;
	unsigned long	no_of_users;
	unsigned long 	curr_level;
	unsigned long 	max_levels;
	struct semaphore res_action_sem;
	struct list_head users_list;
	struct atomic_notifier_head pre_notifier_list[MAX_LEVEL];
	struct atomic_notifier_head post_notifier_list[MAX_LEVEL];
	int linked_res_num;
	char **linked_res_names;
	struct list_head linked_res_handles;
	int (*action)(struct shared_resource *res,
		      unsigned short current_level,
		      unsigned short target_level);
	int (*validate)(struct shared_resource *res,
		unsigned short current_level, unsigned short target_level);
};

struct  users_list {
	const char *usr_name;
	unsigned long 	level;
	struct 	list_head node;
	short		index;
};

/* Shared Resource Framework API's */
int resource_init(struct shared_resource **res_list);
struct res_handle * resource_get(const char *name, const char *id);
int resource_request(struct res_handle *res, unsigned short target_level);
int resource_release(struct res_handle *res);
void resource_put(struct res_handle *res);
void resource_register_pre_notification(struct res_handle *res, struct notifier_block *nb, unsigned short target_level);
void resource_register_post_notification(struct res_handle *res, struct notifier_block *nb, unsigned short target_level);
void resource_unregister_pre_notification(struct res_handle *res, struct notifier_block *nb, unsigned short target_level);
void resource_unregister_post_notification(struct res_handle *res, struct notifier_block *nb, unsigned short target_level);
int  resource_get_level(struct res_handle *res);
int  resource_fixup_level(struct res_handle *res, unsigned long level);

/* Constraints API's */
struct constraint_handle * constraint_get(const char *name,
					  struct constraint_id *id);
int constraint_set(struct constraint_handle *co, unsigned short constraint_level);
int constraint_remove(struct constraint_handle *co);
void constraint_put(struct constraint_handle *co);
void constraint_register_pre_notification(struct constraint_handle *co, struct notifier_block *nb, unsigned short target_level);
void constraint_register_post_notification(struct constraint_handle *co, struct notifier_block *nb, unsigned short target_level);
void constraint_unregister_pre_notification(struct constraint_handle *co, struct notifier_block *nb, unsigned short target_level);
void constraint_unregister_post_notification(struct constraint_handle *co, struct notifier_block *nb, unsigned short target_level);
int constraint_get_level(struct constraint_handle *co);

#endif /* __ARCH_ARM_OMAP_RESOURCE_H */
