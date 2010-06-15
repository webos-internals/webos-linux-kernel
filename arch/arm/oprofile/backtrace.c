/*
 * Arm specific backtracing code for oprofile
 *
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <rpurdie@openedhand.com>
 *
 * Based on i386 oprofile backtrace code by John Levon, David Smith
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/oprofile.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <asm/ptrace.h>
#include <asm/uaccess.h>

#include "../kernel/stacktrace.h"

static int report_trace(struct stackframe *frame, void *d)
{
	unsigned int *depth = d;

	if (*depth) {
		oprofile_add_trace(frame->lr);
		(*depth)--;
	}

	return *depth == 0;
}

/**
 * For history on the abridged vs. apcs (Arm Procedure Call Standard) frame,
 *  see:
 *
 * ARM stack frame simplification (from gcc mailing list)
 * http://gcc.gnu.org/ml/gcc-patches/2008-03/msg00697.html
 */

#undef APCS_FRAMES_ONLY_BACKTRACE

#ifndef APCS_FRAMES_ONLY_BACKTRACE

typedef enum
{
	FRAME_TYPE_INVALID = 0,
	FRAME_TYPE_ABRIDGED_LEAF,
	FRAME_TYPE_ABRIDGED,
	FRAME_TYPE_APCS,
	FRAME_TYPE_END,
} FrameType;

struct AbridgedFrameLeaf
{
	unsigned long dc[3];
	unsigned long fp;
} __attribute__((packed));

struct AbridgedFrame
{
	unsigned long dc[2];
	unsigned long fp;
	unsigned long lr;
} __attribute__((packed));

struct ApcsFrame
{
	unsigned long fp;
	unsigned long sp;
	unsigned long lr;
	unsigned long pc;
} __attribute__((packed));

typedef union {

	struct AbridgedFrameLeaf abr_leaf;
	struct AbridgedFrame     abr;
	struct ApcsFrame         apcs;

	unsigned long buf[4];
} __attribute__((packed)) Frame;

static inline bool
mem_on_stack(unsigned long mem, unsigned long sp)
{
	return mem > sp;
}

static FrameType
copy_user_frame(Frame *frame, unsigned long fp, unsigned long sp)
{
	FrameType ftype;

	int i;
	unsigned long  __user user_ptr;

	user_ptr = fp;
	ftype = FRAME_TYPE_ABRIDGED_LEAF;

	for (i = 3; i >= 0 && user_ptr >= sp;
	     i--, ftype++, user_ptr-=4)
	{
		/* TODO can we speed this up and just do 1 copy instead
		 * of potentially 4? */

		/* check accessibility */
		if (!access_ok(VERIFY_READ, user_ptr, sizeof(unsigned long)))
			return FRAME_TYPE_INVALID;

		if (__copy_from_user_inatomic(&frame->buf[i], (void*)user_ptr,
					sizeof(unsigned long)))
			return FRAME_TYPE_INVALID;

		if (mem_on_stack(frame->buf[i], sp))
		{
			return ftype;
		}
	}

	return FRAME_TYPE_INVALID;
}

static unsigned long user_backtrace_flex(
	unsigned long fp, unsigned long sp, unsigned long lr_reg, unsigned long *ret_lr)
{
	Frame		  frame;
	unsigned long nextfp;
	unsigned long lr;

	FrameType ftype = copy_user_frame(&frame, fp, sp);

	switch (ftype)
	{
		case FRAME_TYPE_ABRIDGED_LEAF:
			nextfp = frame.abr_leaf.fp;
			lr     = lr_reg;
			break;
		case FRAME_TYPE_ABRIDGED:
			nextfp = frame.abr.fp;
			lr     = frame.abr.lr;
			break;
		case FRAME_TYPE_APCS:
			nextfp = frame.apcs.fp;
			lr     = frame.apcs.lr;
			break;
		default:
			return 0;
	}

	*ret_lr = lr;

	if (nextfp <= fp)
		return 0;

	return nextfp;
}

void arm_backtrace(struct pt_regs * const regs, unsigned int depth)
{
	unsigned long fp = regs->ARM_fp;
	unsigned long sp = regs->ARM_sp;
	unsigned long lr = regs->ARM_lr;

	if (!user_mode(regs)) {
		unsigned long base = ((unsigned long)regs) & ~(THREAD_SIZE - 1);
		walk_stackframe(regs->ARM_fp, base, base + THREAD_SIZE,
				report_trace, &depth);
		return;
	}

	while (depth-- && fp && !(fp & 3))
	{
		unsigned long ret_lr = 0;

		fp = user_backtrace_flex(fp, sp, lr, &ret_lr);
		if (ret_lr)
		{
			oprofile_add_trace(ret_lr);
		}
	}
}
#endif

#ifdef APCS_FRAMES_ONLY_BACKTRACE
/*
 * The registers we're interested in are at the end of the variable
 * length saved register structure. The fp points at the end of this
 * structure so the address of this struct is:
 * (struct frame_tail *)(xxx->fp)-1
 */
struct frame_tail {
	struct frame_tail *fp;
	unsigned long sp;
	unsigned long lr;
} __attribute__((packed));

static struct frame_tail* user_backtrace(struct frame_tail *tail)
{
	struct frame_tail buftail[2];

	/* Also check accessibility of one struct frame_tail beyond */
	if (!access_ok(VERIFY_READ, tail, sizeof(buftail)))
		return NULL;
	if (__copy_from_user_inatomic(buftail, tail, sizeof(buftail)))
		return NULL;

	oprofile_add_trace(buftail[0].lr);

	/* frame pointers should strictly progress back up the stack
	 * (towards higher addresses) */
	if (tail >= buftail[0].fp)
		return NULL;

	return buftail[0].fp-1;
}

void arm_backtrace(struct pt_regs * const regs, unsigned int depth)
{
	struct frame_tail *tail = ((struct frame_tail *) regs->ARM_fp) - 1;

	if (!user_mode(regs)) {
		unsigned long base = ((unsigned long)regs) & ~(THREAD_SIZE - 1);
		walk_stackframe(regs->ARM_fp, base, base + THREAD_SIZE,
				report_trace, &depth);
		return;
	}

	while (depth-- && tail && !((unsigned long) tail & 3))
		tail = user_backtrace(tail);
}
#endif
