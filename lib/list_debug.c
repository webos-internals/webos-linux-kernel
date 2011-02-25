/*
 * Copyright 2006, Red Hat, Inc., Dave Jones
 * Released under the General Public License (GPL).
 *
 * This file contains the linked list implementations for
 * DEBUG_LIST.
 */

#include <linux/module.h>
#include <linux/list.h>

/*
 * Insert a new entry between two known consecutive entries.
 *
 * This is only for internal list manipulation where we know
 * the prev/next entries already!
 */

void __list_add(struct list_head *new,
			      struct list_head *prev,
			      struct list_head *next)
{
	int bug = 0;

	if (unlikely(next->prev != prev)) {
		printk(KERN_ERR "list_add corruption. next->prev should be "
			"prev (%p), but was %p. (next=%p).\n",
			prev, next->prev, next);
		bug++;
	}
	if (unlikely(prev->next != next)) {
		printk(KERN_ERR "list_add corruption. prev->next should be "
			"next (%p), but was %p. (prev=%p).\n",
			next, prev->next, prev);
		bug++;
	}

#ifdef CONFIG_DEBUG_FREE_PAGE_LIST_PALM
	if (unlikely(LIST_DEBUG_MAGIC_IN_FREE_LIST == new->magic)) {
		printk(KERN_ERR "###############################################################\n");
		printk(KERN_ERR "##### Adding element that resides in free list to another list:\n");
		printk(KERN_ERR "  new:       %p\n", new);
		printk(KERN_ERR "  new->next: %p\n", new->next);
		printk(KERN_ERR "  new->prev: %p\n", new->prev);
		if (!bug)
			dump_stack();
	}
#endif

	if (bug) {
		dump_stack();
		BUG();
	}

	next->prev = new;
	new->next = next;
	new->prev = prev;
	prev->next = new;
}
EXPORT_SYMBOL(__list_add);

/**
 * list_add - add a new entry
 * @new: new entry to be added
 * @head: list head to add it after
 *
 * Insert a new entry after the specified head.
 * This is good for implementing stacks.
 */
void list_add(struct list_head *new, struct list_head *head)
{
	__list_add(new, head, head->next);
}
EXPORT_SYMBOL(list_add);

/**
 * list_del - deletes entry from list.
 * @entry: the element to delete from the list.
 * Note: list_empty on entry does not return true after this, the entry is
 * in an undefined state.
 */
void list_del(struct list_head *entry)
{
	int bug = 0;

	if (unlikely(entry->prev->next != entry)) {
		printk(KERN_ERR "list_del corruption. prev->next should be %p, "
				"but was %p\n", entry, entry->prev->next);
		bug++;
	}
	if (unlikely(entry->next->prev != entry)) {
		printk(KERN_ERR "list_del corruption. next->prev should be %p, "
				"but was %p\n", entry, entry->next->prev);
		bug++;
	}

	if (bug) {
		dump_stack();
		BUG();
	}

	__list_del(entry->prev, entry->next);
	entry->next = LIST_POISON1;
	entry->prev = LIST_POISON2;
}
EXPORT_SYMBOL(list_del);
