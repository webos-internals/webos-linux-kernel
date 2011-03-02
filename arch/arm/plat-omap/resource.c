/*
 * OMAP34XX Shared Resource Framework
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
 * History:
 *
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <asm/arch/resource.h>
#include <asm/arch/prcm.h>

//#define DEBUG_PRCM 1

#ifdef DEBUG_PRCM
# define DPRINTK(fmt, args...)	\
	printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)
#else
#  define DPRINTK(fmt, args...)
#endif

static	DEFINE_SPINLOCK(res_handle_lock);
static	DEFINE_SPINLOCK(users_list_lock);

#define down_srfmutex(x)	{\
				if (!(in_atomic() || irqs_disabled())) \
					down(x);\
				}
#define up_srfmutex(x)		{\
				if (!(in_atomic() || irqs_disabled())) \
					up(x);\
				}

/* Pool of resource handles */
static short  handle_flag[MAX_HANDLES];
static struct res_handle handle_list[MAX_HANDLES];

/* Pool of users for a resource */
static short  usr_flag[MAX_USERS];
static struct users_list usr_list[MAX_USERS];

/* Pool of linked resource handles */
static struct res_handle_node linked_hlist[MAX_HANDLES];

/* Global data to share across functions */
static struct shared_resource **res_list;

/* This is the initializtion function for Shared resource framework. */
/* Initializes all the members of res_list. Always returns 0.	     */
int __init resource_init(struct shared_resource **resource_list)
{
	struct shared_resource **resp;
	int ind;
	int i;
	int linked_cnt = 0;

	res_list = resource_list;

	printk("OMAP: Initializing Shared Resource Framework\n");

	/* Init the res_handle POOL */
	for (ind = 0; ind < MAX_HANDLES; ind++) {
		handle_flag[ind]           = RES_UNUSED;
		handle_list[ind].res       = NULL;
		handle_list[ind].usr_name  = NULL;
		handle_list[ind].res_index = ind;
		handle_list[ind].usr_index = -1;
	}

	/* Init the users_list POOL */
	for (ind = 0; ind < MAX_USERS; ind++) {
		usr_flag[ind]          = RES_UNUSED;
		usr_list[ind].usr_name = NULL;
		usr_list[ind].level    = DEFAULT_LEVEL;
		usr_list[ind].index    = ind;
	}

	/* Init all the resources in  res_list */
	for (resp = res_list; *resp; resp++) {
		INIT_LIST_HEAD(&(*resp)->users_list);
		init_MUTEX(&(*resp)->res_action_sem);
		INIT_LIST_HEAD(&(*resp)->linked_res_handles);

		/*
		 * latency resources have to follow domain resources in
		 * res_list!
		 */
		if (strncmp((*resp)->name, "lat_", 4) == 0) {
			for (i = 0; i < (*resp)->linked_res_num; i++) {
				if (linked_cnt >= MAX_HANDLES)
					goto res_init_fail;
				linked_hlist[linked_cnt].handle =
						resource_get((*resp)->name,
						(*resp)->linked_res_names[i]);

				if (IS_ERR(linked_hlist[linked_cnt].handle))
						goto res_init_fail;

				list_add(&linked_hlist[linked_cnt].node,
						 &(*resp)->linked_res_handles);
				linked_cnt++;
			}
		}
	}
	return 0;

res_init_fail:
	/* TODO: free all memory and resources */
	printk(KERN_WARNING "WARNING: Leaking memory in failed resource_init()\n");
	return -1;
}

/* Returns a res_handle structure to the caller, which has a resource*/
/* structure and a name which is the clock name embedded in it. The clock */
/* name is embedded to track futher calls by the same device.*/
struct res_handle *resource_get(const char *name, const char *id)
{
	struct shared_resource **resp;
	short index = 0;
	struct res_handle *res = ERR_PTR(-ENOENT);
	unsigned long flags;

	if (name == NULL || id == NULL) {
		printk(KERN_ERR "resource_get: Invalid pointer\n");
		return res;
	}
	DPRINTK("resource_get for %s Clock-name %s\n", id, name);
	spin_lock_irqsave(&res_handle_lock, flags);
	for (resp = res_list; *resp; resp++) {
		if (strcmp((*resp)->name, id) == 0) {
			/* look for a free handle from the handle_list POOL */
			while (index < MAX_HANDLES) {
				if (handle_flag[index] == RES_UNUSED)
					break;

				index++;
			}
			if (index >= MAX_HANDLES) {
				/* No free handle available */
				panic("FATAL ERROR: res_handle POOL EMPTY\n");
			}
			handle_flag[index] = RES_USED;
			handle_list[index].res = (*resp);
			handle_list[index].usr_name = name;
			handle_list[index].res_index = index;
			DPRINTK("Returning the handle for %s, index = %d\n",
				id, index);
			res = &handle_list[index];
			break;
		}
	}
	spin_unlock_irqrestore(&res_handle_lock, flags);
	return res;
}

/* Adds the request to the list of requests for the given resource. Recalulates
 * the target level to be set for the resource and updates it if not same as
 * the current level. Also calls notification functions for registered users to
 * notify the target level change. */
int resource_request(struct res_handle *res, unsigned short target_level)
{
	struct 	shared_resource *resp;
	struct 	users_list *user, *cur_user = NULL;
	short 	index = 0;
	int	ret = -1;
	unsigned long flags;

	if (res == ERR_PTR(-ENOENT)) {
		DPRINTK("Invalid resource handle passed to reource_request\n");
		return ret;
	}

	resp = res->res;

	DPRINTK("resource_request: Clock-name %s\n", res->usr_name);

	spin_lock_irqsave(&users_list_lock, flags);

	if (res->usr_index != -1) {
		cur_user = &usr_list[res->usr_index];
		ret = resp->validate(resp, cur_user->level, target_level);
		if (ret) {
			DPRINTK("Validation failed\n");
			ret = -EINVAL;
			goto out_unlock;
		}
		DPRINTK("Updating resource level for %s, to %d\n",
			resp->name, target_level);
		cur_user->level = target_level;

	} else {
		ret = resp->validate(resp, DEFAULT_LEVEL, target_level);
		if (ret) {
			DPRINTK("Validation failed\n");
			ret = -EINVAL;
			goto out_unlock;
		}

		DPRINTK("Adding resource level for %s, to %d\n",
			resp->name, target_level);
		while (index < MAX_USERS) {
			if (usr_flag[index] == RES_UNUSED)
				break;
			index++;
		}
		if (index >= MAX_USERS)
			panic("FATAL ERROR: users_list POOL EMPTY\n");

		usr_flag[index]          = RES_USED;
		usr_list[index].usr_name = res->usr_name;
		usr_list[index].level    = target_level;
		usr_list[index].index    = index;
		res->usr_index = index;
		list_add(&usr_list[index].node, &resp->users_list);
		cur_user = &usr_list[index];
		/* Increment the number of users for this domain. */
		resp->no_of_users++;
	}

	if (target_level == resp->curr_level)
		goto out_unlock;

	/* Regenerate the target_level for the resource:
	 *   Check if any other user of that resource has requested a higher
	 *   constraint. */
	target_level = DEFAULT_LEVEL;
	list_for_each_entry(user, &(resp->users_list), node) {
		if (user->level > target_level)
			target_level = user->level;
	}
	spin_unlock_irqrestore(&users_list_lock, flags);
	DPRINTK("New Target level is %d\n", target_level);

	if (target_level != resp->curr_level) {
		DPRINTK("Changing Level for resource %s to %d\n",
			resp->name, target_level);
		if ((resp->res_type != RES_FREQ_CO) &&
				(target_level < MAX_LEVEL)) {
			/* Call the notification functions */
			/* Currently not supported for Frequency constraints */
			atomic_notifier_call_chain(
				&resp->pre_notifier_list[target_level],
							target_level, NULL);
		}
		down_srfmutex(&resp->res_action_sem);
		ret = resp->action(resp, resp->curr_level, target_level);
		up_srfmutex(&resp->res_action_sem);
		if (ret)
			panic("FATAL ERROR: Unable to Change "
						"level for resource %s to %d\n",
				resp->name, target_level);
		else
			/* If successful, change the resource curr_level */
			resp->curr_level = target_level;
		if ((resp->res_type != RES_FREQ_CO) &&
				(target_level < MAX_LEVEL))
			atomic_notifier_call_chain(
				&resp->post_notifier_list[target_level],
						target_level, NULL);
	} else {
		/* return success */
		ret = 0;
	}

	return ret;
out_unlock:
	spin_unlock_irqrestore(&users_list_lock, flags);
	return ret;
}

/* Deletes the request from the list of request for the given resource.
 * Recalulates the target level to be set for the resource and updates it if
 * not same as the current level.Also calls notification functions for
 * registered users to notify the target level change			
 */
int resource_release(struct res_handle *res)
{
	struct  shared_resource *resp;
	struct  users_list *user, *cur_user = NULL;
	unsigned short target_level;
	int ret = 0;
	unsigned long flags;

	if (res == ERR_PTR(-ENOENT)) {
		DPRINTK("Invalid resource handle passed to reource_release\n");
		return ret;
	}

	resp = res->res;
	DPRINTK("resource_request: Clock-name %s\n", res->usr_name);

	spin_lock_irqsave(&users_list_lock, flags);

	if (res->usr_index == -1)
		goto out_unlock;

	cur_user = &usr_list[res->usr_index];

	ret = resp->validate(resp, cur_user->level, DEFAULT_LEVEL);
	if (ret) {
		DPRINTK("Validation failed\n");
		ret = -EINVAL;
		goto out_unlock;
	}

	/* Delete the resource */
	DPRINTK("Deleting resource for %s\n", resp->name);
	list_del(&cur_user->node);
	usr_list[cur_user->index].usr_name = NULL;
	usr_list[cur_user->index].level    = DEFAULT_LEVEL;
	usr_list[cur_user->index].index    = cur_user->index;
	usr_flag[cur_user->index]          = RES_UNUSED;
	res->usr_index = -1;

	/* Decrement the number of users for this domain. */
	resp->no_of_users--;

	/* Regenerate the target_level for the resource */
	target_level = DEFAULT_LEVEL;
	list_for_each_entry(user, &(resp->users_list), node) {
		if (user->level > target_level)
			target_level = user->level;
	}

	spin_unlock_irqrestore(&users_list_lock, flags);
	DPRINTK("New Target level is %d\n", target_level);

	if ((target_level == DEFAULT_LEVEL) &&
			(resp->prcm_id == PRCM_VDD1_CONSTRAINT))
		target_level = CO_VDD1_OPP1;
	else if ((target_level == DEFAULT_LEVEL) &&
			 (resp->prcm_id == PRCM_VDD2_CONSTRAINT))
		target_level = CO_VDD2_OPP2;
	else if ((target_level == DEFAULT_LEVEL) &&
			(resp->prcm_id == PRCM_ARMFREQ_CONSTRAINT))
		target_level = min_arm_freq;
	else if ((target_level == DEFAULT_LEVEL) &&
			(resp->prcm_id == PRCM_DSPFREQ_CONSTRAINT))
		target_level = min_dsp_freq;

	/* No level change. Get out... */
	if (target_level == resp->curr_level)
		return 0;


	DPRINTK("Changing Level for resource %s to %d\n",
		resp->name, target_level);
	if ((resp->res_type != RES_FREQ_CO) && (target_level < MAX_LEVEL))
		atomic_notifier_call_chain(
				&resp->pre_notifier_list[target_level],
				target_level, NULL);

	down_srfmutex(&resp->res_action_sem);
	ret = resp->action(resp, resp->curr_level, target_level);
	up_srfmutex(&resp->res_action_sem);

	if (ret)
		printk(KERN_ERR "FATAL ERROR: Unable to Change "
				"level for resource %s to %d\n",
					resp->name, target_level);
	else
		/* If successful, change the resource curr_level */
		resp->curr_level = target_level;

	/* HACK... OR NOT... ALWAYS CHANGE LEVEL... */
	resp->curr_level = target_level;

	if ((resp->res_type != RES_FREQ_CO) && (target_level < MAX_LEVEL))
		atomic_notifier_call_chain(
				&resp->post_notifier_list[target_level],
				target_level, NULL);
	return 0;

out_unlock:
	spin_unlock_irqrestore(&users_list_lock, flags);
	return ret;
}

/* Frees the res_handle structure from the pool */
void resource_put(struct res_handle *res)
{
	unsigned long flags;
	if (res == ERR_PTR(-ENOENT)) {
		DPRINTK("Invalid resource handle passed to resource_put\n");
		return;
	}

	if (res->usr_index != -1) {
		printk(KERN_ERR "resource_put called before "
				"resource_release\n");
		return;
	}
	DPRINTK("resource_put for %s, index = %d\n",
			res->res->name, res->res_index);
	spin_lock_irqsave(&res_handle_lock, flags);
	handle_flag[res->res_index] = RES_UNUSED;
	handle_list[res->res_index].res = NULL;
	handle_list[res->res_index].usr_name = NULL;
	handle_list[res->res_index].res_index = res->res_index;
	handle_list[res->res_index].usr_index = -1;
	spin_unlock_irqrestore(&res_handle_lock, flags);
}

/* Returns the current level for a resource */
int resource_get_level(struct res_handle *res)
{
	struct shared_resource *resp;
	int ret;
	unsigned long flags;

	resp = res->res;
	spin_lock_irqsave(&res_handle_lock, flags);
	ret = resp->curr_level;
	spin_unlock_irqrestore(&res_handle_lock, flags);
	return ret;
}

/* Registers a notification function from a resource user for a specific  */
/* target level. The function is called before a level change for the     */
/* resource to the target level.					  */
void resource_register_pre_notification(struct res_handle *res,
			struct notifier_block *nb, unsigned short target_level)
{
	struct shared_resource *resp;
	int ind;

	resp = res->res;
	DPRINTK("Notification registered for %s, level %d\n",
		resp->name, target_level);
	if (target_level == resp->max_levels) {
		for (ind = DEFAULT_LEVEL; ind < resp->max_levels; ind++) {
			res->nb_internal_pre[ind] =
				(struct notifier_block *)kmalloc
				(sizeof(struct notifier_block), GFP_KERNEL);
			*(res->nb_internal_pre[ind]) = *nb;
			atomic_notifier_chain_register(
				&resp->pre_notifier_list[ind],
				res->nb_internal_pre[ind]);
		}
	} else {
		res->nb_internal_pre[target_level] =
			(struct notifier_block *)
			kmalloc(sizeof(struct notifier_block), GFP_KERNEL);
		*(res->nb_internal_pre[target_level]) = *nb;
		atomic_notifier_chain_register(
				&resp->pre_notifier_list[target_level],
					res->nb_internal_pre[target_level]);
	}
}


/* Registers a notification function from a resource user for a specific  */
/* target level. The function is called before a level change for the     */
/* resource to the target level.					  */
void resource_register_post_notification(struct res_handle *res,
			struct notifier_block *nb, unsigned short target_level)
{
	struct shared_resource *resp;
	int ind;

	resp = res->res;
	DPRINTK("Notification registered for %s, level %d\n",
		resp->name, target_level);
	if (target_level == resp->max_levels) {
		for (ind = DEFAULT_LEVEL; ind < resp->max_levels; ind++) {
			res->nb_internal_post[ind] =
				(struct notifier_block *)
				kmalloc(sizeof(struct notifier_block),
								GFP_KERNEL);
			*(res->nb_internal_post[ind]) = *nb;
			atomic_notifier_chain_register(
				&resp->post_notifier_list[ind],
				res->nb_internal_post[ind]);
		}
	} else {
		res->nb_internal_post[target_level] =
			(struct notifier_block *)
			kmalloc(sizeof(struct notifier_block), GFP_KERNEL);
		*(res->nb_internal_post[target_level]) = *nb;
		atomic_notifier_chain_register(
			&resp->post_notifier_list[target_level],
			res->nb_internal_post[target_level]);
	}
}


/* Unregisters the notification function from a resource user for a specific  */
/* target level.    							      */
void resource_unregister_pre_notification(struct res_handle *res,
			struct notifier_block *nb, unsigned short target_level)
{
	struct shared_resource *resp;
	int ind, ret;

	resp = res->res;
	DPRINTK("Notification unregistered for %s, level %d\n",
		resp->name, target_level);
	if (target_level == resp->max_levels) {
		for (ind = DEFAULT_LEVEL; ind < resp->max_levels; ind++) {
			ret = atomic_notifier_chain_unregister(
				&resp->pre_notifier_list[ind],
				res->nb_internal_pre[ind]);
			if (ret != -ENOENT)
				kfree(res->nb_internal_pre[ind]);
		}
	} else {
		ret = atomic_notifier_chain_unregister(
			&resp->pre_notifier_list[target_level],
			res->nb_internal_pre[target_level]);
		if (ret != -ENOENT)
			kfree(res->nb_internal_pre[target_level]);
	}
}


/* Unregisters the notification function from a resource user for a specific  */
/* target level.    							      */
void resource_unregister_post_notification(struct res_handle *res,
			struct notifier_block *nb, unsigned short target_level)
{
	struct shared_resource *resp;
	int ind, ret;

	resp = res->res;
	DPRINTK("Notification unregistered for %s, level %d\n",
		resp->name, target_level);
	if (target_level == resp->max_levels) {
		for (ind = DEFAULT_LEVEL; ind < resp->max_levels; ind++) {
			ret = atomic_notifier_chain_unregister(
				&resp->post_notifier_list[ind],
				res->nb_internal_post[ind]);
			if (ret != -ENOENT)
				kfree(res->nb_internal_post[ind]);
		}
	} else {
		ret = atomic_notifier_chain_unregister(
				&resp->post_notifier_list[target_level],
				res->nb_internal_post[target_level]);
		if (ret != -ENOENT)
			kfree(res->nb_internal_post[target_level]);
	}
}

