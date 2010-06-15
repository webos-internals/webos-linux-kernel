/*
 * linux/fs/minicore.c
 *
 * Deprecated, but this would be a good place to experiment
 * with backtrace with fp in order to get oprofile working.
 */


//#define MINICORE_NO_MAPS
#define MINICORE_NO_BACKTRACE
//#define MINICORE_DEBUG_BACKTRACE
//#define MINICORE_FRAMETAIL_NO_SP


#include <linux/module.h>

#include <linux/file.h>
#include <linux/mm.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <asm/uaccess.h>


// defined in exec.c
#define CORENAME_MAX_SIZE 64
int format_corename(char *corename, const char *pattern, long signr, siginfo_t *info);

// linux/fs/proc/task_mmu.c
int show_map(struct seq_file *m, void *v);


char minicore_pattern[65] = "/var/minicores/minicore.%p.%e";


struct crashlog
{
	char        *buffer;
	int          size;
	struct file *file;
};


static void crashlog_reset(struct crashlog *log)
{
    log->size = 0;
}


static int crashlog_dump(struct crashlog *log)
{
    struct file *file;
    int ret;

    file = log->file;
    ret = file->f_op->write(file, log->buffer, log->size, &file->f_pos);
    return ret;
}


static void crashlog_write(struct crashlog *log, const char *msg,
	unsigned int count)
{
    while (count--)
    {
        char ch = *msg++;

        if (0 == ch)
        {
            break;
        }

        log->buffer[log->size++] = ch;

        if (log->size >= PAGE_SIZE)
        {
            crashlog_dump(log);
            crashlog_reset(log);
        }
    }

    log->buffer[log->size] = 0;
    return;
}


/**
 * Print to crashlog null terminated string.
 *
 * Formatted string has maximum length of 1024.
 */
static void crashlog_printk(struct crashlog *log, const char *fmt, ...)
{
    va_list args;
    char buf[1024];
    int len;

    /* Emit the output into the temporary buffer */
    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    crashlog_write(log, buf, len);
}


#define iferr_goto(cond, label) \
    do {                                                               \
        if (cond)                                                      \
        {                                                              \
            printk("%s: Condition " #cond " failed!\n", __FUNCTION__); \
            goto label;                                                \
        }                                                              \
    } while (0)


static int minicore_open_file(const char * const path, int flags, int mode,
	struct file **ret_file)
{
    struct file *file;
    struct inode *inode;

    file = filp_open(path, flags, mode);
    if (IS_ERR(file))
    {
        printk("path %s because of %ld\n", path, (unsigned long)file);
        goto fail;
    }

    inode = file->f_dentry->d_inode;

    /* multiple links - don't dump */
    iferr_goto(inode->i_nlink > 1, close_fail);

    iferr_goto(d_unhashed(file->f_dentry), close_fail);

    iferr_goto(!S_ISREG(inode->i_mode), close_fail);

    iferr_goto(!file->f_op, close_fail);
    iferr_goto(!file->f_op->write, close_fail);

    iferr_goto(do_truncate(file->f_dentry, 0, 0, file) != 0, close_fail);

    *ret_file = file;

    return 0;

close_fail:
    filp_close(file, NULL);
fail:
    *ret_file = NULL;
    return -EIO;
}


static void minicore_close_file(struct file *file)
{
    filp_close(file, NULL);
}


static int crashlog_init(struct crashlog *log, const char * const path)
{
    int ret;
    struct file *file;

    if (!log)
        return -EFAULT;

    if (minicore_open_file(path, O_CREAT | O_RDWR, 0644, &file))
    {
        ret = -EIO;
        goto err_file;
    }

    log->size = 0;
    log->file = file;

    /* avoid sleeping while get page */
    log->buffer = (char*)__get_free_page(GFP_ATOMIC);
    if (!log->buffer)
    {
        ret = -ENOMEM;
        goto err_mem;
    }

    return 0;

err_mem:
    filp_close(log->file, NULL);
err_file:
    return ret;
}


static void crashlog_deinit(struct crashlog *log)
{
    if (log->file)
    {
        minicore_close_file(log->file);
    }

    free_page((unsigned long)log->buffer);
}


struct vma_info
{
	unsigned long stack_vm_start;
	unsigned long stack_vm_end;
};


#ifndef MINICORE_NO_MAPS


/**
 * Equivalent of reading the /proc/<pid>/maps file.
 */
static int record_maps(struct crashlog *log, struct task_struct *task,
	struct vma_info *vma_info)
{
    int retval;
    char *buf;
    struct mm_struct *mm;
    struct vm_area_struct *vma;
    struct seq_file m;
    int n;

    crashlog_printk(log, "VmaMaps:\n");

    retval = 0;

    /* avoid sleeping while get page */
    buf = (char*)__get_free_page(GFP_ATOMIC);
    if (!buf)
    {
        retval = -ENOMEM;
        goto out;
    }

    //---------------------------------------------------

    mm = get_task_mm(task);
    if (!mm)
    {
        retval = -ENOEXEC;
        goto out2;
    }
    down_read(&mm->mmap_sem);

	//	see also mm-> start_code, end_code, start_data, end_data;
	//	   start_brk, brk, start_stack;
	//vma_info->start_stack = mm->start_stack;
	
    vma = mm->mmap;
    while (vma)
    {
        memset(&m, 0, sizeof(m));
        m.private = task;
        m.buf = buf;
        m.size = PAGE_SIZE;

		if (vma->vm_start <= mm->start_stack &&
			vma->vm_end >= mm->start_stack)
		{
			vma_info->stack_vm_start = vma->vm_start;
			vma_info->stack_vm_end   = vma->vm_end;
		}
	
		// use the show_map function from linux/fs/proc/task_mmu.c
		// so the format is the same as /proc/<pid>/maps
        n = show_map(&m, vma);
        if (n < 0)
        {
            retval = -ENOMSG;
            break;
        }

        if (m.count == m.size)
        {
            retval = -E2BIG;
            break;
        }

        crashlog_write(log, buf, m.count);

        vma = vma->vm_next;
    }

    up_read(&mm->mmap_sem);
    mmput(mm);

    //---------------------------------------------------

out2:
    free_page((unsigned long)buf);

out:

    crashlog_printk(log, "\n");
    return retval;
}


#endif // MINICORE_NO_MAPS


static void crashlog_printbtlink(struct crashlog *log, int depth,
	unsigned long addr)
{
    crashlog_printk(log, "#%-2d 0x%08lx\n", depth, addr);
}


#ifndef MINICORE_NO_BACKTRACE


// see linux/arch/arm/oprofile/backtrace.c for reference

/*
 * The registers we're interested in are at the end of the variable
 * length saved register structure. The fp points at the end of this
 * structure so the address of this struct is:
 * (struct frame_tail *)(xxx->fp)-1
 */
struct frame_tail {
    struct frame_tail *fp;
#ifndef MINICORE_FRAMETAIL_NO_SP
    unsigned long sp;
#endif
    unsigned long lr;
} __attribute__((packed));


#ifndef MINICORE_FRAMETAIL_NO_SP
	#define adjust_frame_tail(fp_)	(fp_)
#else
	static struct frame_tail* adjust_frame_tail(const struct frame_tail *fp)
	{
		return (struct frame_tail*)(((unsigned long)fp) + 4);
	}
#endif


#ifdef MINICORE_DEBUG_BACKTRACE
static void dump_usermem(struct crashlog *log, unsigned long p, int iStart,
	int iEnd)
{
	int i;
	unsigned long addr;
	unsigned long n;

	for (i = iStart; i <= iEnd; i++)
	{
		addr = p + i * 4;

		if (__copy_from_user_inatomic(&n, (const void*)addr, sizeof(n)))
		{
			crashlog_printk(log, " <0x%08lx>: error\n", addr);
			break;
		}	
		
		crashlog_printk(log, "%c<0x%08lx>: 0x%08lx\n", (i == 0) ? '*' : ' ',
			addr, n);
	}
}
#endif


static struct frame_tail* user_backtrace(const struct frame_tail *tail,
	struct crashlog *log, int depth)
{
    struct frame_tail buftail[2];
    unsigned long fp_addr;

    /* Also check accessibility of one struct frame_tail beyond */
    if (!access_ok(VERIFY_READ, tail, sizeof(buftail)))
        return NULL;

	if (__copy_from_user_inatomic(buftail, tail, sizeof(buftail)))
		return NULL;

	#ifdef MINICORE_DEBUG_BACKTRACE

		crashlog_printk(log, "\n");

		crashlog_printk(log, "%d mem around frame_tail:\n", depth);
		dump_usermem(log, tail, -4, 4);

		crashlog_printk(log, "%d frame_tail.fp = 0x%08lx\n", depth,
			(unsigned long)buftail[0].fp);
	
		#ifndef MINICORE_FRAMETAIL_NO_SP
			crashlog_printk(log, "%d frame_tail.sp = 0x%08lx\n", depth, buftail[0].sp);
		#endif
	
		crashlog_printk(log, "%d frame_tail.lr = 0x%08lx\n", depth, buftail[0].lr);
	#endif
	
    fp_addr = buftail[0].lr;
    crashlog_printbtlink(log, depth, fp_addr);

    /* frame pointers should strictly progress back up the stack
     * (towards higher addresses) */
    if (tail >= buftail[0].fp)
        return NULL;

    return adjust_frame_tail(buftail[0].fp) - 1;
}


#endif // MINICORE_NO_BACKTRACE


static void find_stacktrace(struct crashlog* log, const struct task_struct *task,
	const struct pt_regs* regs_, const struct vma_info *vma_info)
{
#ifndef MINICORE_NO_BACKTRACE
    const int maxDepth = 20;
#endif

#ifndef MINICORE_NO_BACKTRACE
    const struct frame_tail *tail;
#endif
    const struct pt_regs *regs;
    int depth;
//	unsigned long last_address;

    // see linux/arch/arm/kernel/ptrace.c get_user_regs for reference
    // or linux/arch/arm/kernel/process.c dump_task_regs et al
    #if 1
		// the passed in regs_ should give the correct value
		regs = regs_;
	#else
		// or we could calculate it ourself
		regs = task_pt_regs(task);
	#endif

#ifdef MINICORE_DEBUG_BACKTRACE

	crashlog_printk(log, "THREAD_SIZE    = 0x%08lx\n", (unsigned long)THREAD_SIZE);
	crashlog_printk(log, "sizeof pt_regs = 0x%08lx\n", (unsigned long)sizeof(struct pt_regs));
	crashlog_printk(log, "\n");

	crashlog_printk(log, "task           = 0x%08lx\n", (unsigned long)task);
	crashlog_printk(log, "task.stack     = 0x%08lx\n", (unsigned long)task->stack);
	crashlog_printk(log, "regs_          = 0x%08lx\n", (unsigned long)regs_);
	crashlog_printk(log, "regs           = 0x%08lx\n", (unsigned long)regs);
	crashlog_printk(log, "\n");

	crashlog_printk(log, "regs.fp        = 0x%08lx\n", (unsigned long)regs->ARM_fp);
	crashlog_printk(log, "regs.sp        = 0x%08lx\n", (unsigned long)regs->ARM_sp);
	crashlog_printk(log, "regs.lr        = 0x%08lx\n", (unsigned long)regs->ARM_lr);
	crashlog_printk(log, "regs.pc        = 0x%08lx\n", (unsigned long)regs->ARM_pc);
	crashlog_printk(log, "\n");

//	crashlog_printk(log, "vma_info.start_stack    = 0x%08lx\n", (unsigned long)vma_info->start_stack);
	crashlog_printk(log, "vma_info.stack_vm_start = 0x%08lx\n", (unsigned long)vma_info->stack_vm_start);
	crashlog_printk(log, "vma_info.stack_vm_end   = 0x%08lx\n", (unsigned long)vma_info->stack_vm_end);
	crashlog_printk(log, "\n");

	crashlog_printk(log, "mem around sp:\n");
	dump_usermem(log, regs->ARM_sp, -256, 256);
	crashlog_printk(log, "\n");

	crashlog_printk(log, "mem around fp:\n");
	dump_usermem(log, regs->ARM_fp, -256, 256);
	crashlog_printk(log, "\n");

#endif

    crashlog_printk(log, "Backtrace:\n");

    depth = 0;
    crashlog_printbtlink(log, depth, instruction_pointer(regs));
    depth++;
	
#ifndef MINICORE_NO_BACKTRACE
    tail = adjust_frame_tail(((struct frame_tail *) regs->ARM_fp)) - 1;

	if (!user_mode(regs))
	{
		crashlog_printk(log, "Error: not user_mode\n");
	}
	else
	{
		//last_address = 0;
		while ((depth < maxDepth) && tail && !((unsigned long) tail & 3))
		{
			//last_address = (unsigned long) tail;
			tail = user_backtrace(tail, log, depth);
			depth++;
		}
	}
#endif

    crashlog_printk(log, "\n");
}


static const char* signal_name_str(int signr)
{
	#define DEFINE_SIG_STR(e)	\
		case e: return #e
	
	switch (signr)
	{
		DEFINE_SIG_STR( /*  1 */ SIGHUP );
		DEFINE_SIG_STR( /*  2 */ SIGINT );
		DEFINE_SIG_STR( /*  3 */ SIGQUIT );
		DEFINE_SIG_STR( /*  4 */ SIGILL );
		DEFINE_SIG_STR( /*  5 */ SIGTRAP );
		DEFINE_SIG_STR( /*  6 */ SIGABRT );
		DEFINE_SIG_STR( /*  7 */ SIGBUS );
		DEFINE_SIG_STR( /*  8 */ SIGFPE );
		DEFINE_SIG_STR( /*  9 */ SIGKILL );
		DEFINE_SIG_STR( /* 10 */ SIGUSR1 );
		DEFINE_SIG_STR( /* 11 */ SIGSEGV );
		DEFINE_SIG_STR( /* 12 */ SIGUSR2 );
		DEFINE_SIG_STR( /* 13 */ SIGPIPE );
		DEFINE_SIG_STR( /* 14 */ SIGALRM );
		DEFINE_SIG_STR( /* 15 */ SIGTERM );
		DEFINE_SIG_STR( /* 16 */ SIGSTKFLT );
		DEFINE_SIG_STR( /* 17 */ SIGCHLD );
		DEFINE_SIG_STR( /* 18 */ SIGCONT );
		DEFINE_SIG_STR( /* 19 */ SIGSTOP );
		DEFINE_SIG_STR( /* 20 */ SIGTSTP );
		DEFINE_SIG_STR( /* 21 */ SIGTTIN );
		DEFINE_SIG_STR( /* 22 */ SIGTTOU );
		DEFINE_SIG_STR( /* 23 */ SIGURG );
		DEFINE_SIG_STR( /* 24 */ SIGXCPU );
		DEFINE_SIG_STR( /* 25 */ SIGXFSZ );
		DEFINE_SIG_STR( /* 26 */ SIGVTALRM );
		DEFINE_SIG_STR( /* 27 */ SIGPROF );
		DEFINE_SIG_STR( /* 28 */ SIGWINCH );
		DEFINE_SIG_STR( /* 29 */ SIGIO );
		DEFINE_SIG_STR( /* 30 */ SIGPWR );
		DEFINE_SIG_STR( /* 31 */ SIGSYS	);
		default:
			break;
	}
	
	#undef DEFINE_SIG_STR

	return "SIG?";
}


// copied from fs/proc/base.c.  we could remove the static there
// but it's a small piece of code so easier to just dupe it.
static int proc_pid_cmdline(struct task_struct *task, char * buffer)
{
	int res = 0;
	unsigned int len;
	struct mm_struct *mm = get_task_mm(task);
	if (!mm)
		goto out;
	if (!mm->arg_end)
		goto out_mm;	/* Shh! No looking before we're done */

 	len = mm->arg_end - mm->arg_start;
 
	if (len > PAGE_SIZE)
		len = PAGE_SIZE;
 
	res = access_process_vm(task, mm->arg_start, buffer, len, 0);

	// If the nul at the end of args has been overwritten, then
	// assume application is using setproctitle(3).
	if (res > 0 && buffer[res-1] != '\0' && len < PAGE_SIZE) {
		len = strnlen(buffer, res);
		if (len < res) {
		    res = len;
		} else {
			len = mm->env_end - mm->env_start;
			if (len > PAGE_SIZE - res)
				len = PAGE_SIZE - res;
			res += access_process_vm(task, mm->env_start, buffer+res, len, 0);
			res = strnlen(buffer, res);
		}
	}
out_mm:
	mmput(mm);
out:
	return res;
}


static void record_cause(struct crashlog* log, const struct task_struct *task,
	int signr)
{
	char cmdLine[PAGE_SIZE];
	int res;
	
    crashlog_printk(log, "Cmd: %s\n", task->comm);

    memset(&cmdLine, 0, sizeof(cmdLine));
	res = proc_pid_cmdline((struct task_struct *)task, cmdLine);
	cmdLine[sizeof(cmdLine) - 1] = 0;
	// note: cmd args should also be retrieved, with null separators,
	// but we only want the exe path here
    crashlog_printk(log, "CmdExe: %s\n", (res <= 0) ? "?" : cmdLine);
	
    crashlog_printk(log, "Pid: %d\n", task->tgid);
    crashlog_printk(log, "Signal: %d\n", signr);
    crashlog_printk(log, "SignalName: %s\n", signal_name_str(signr));
    crashlog_printk(log, "\n");
}


static int dump_minicore_format(const struct task_struct *task, int signr,
	const struct pt_regs *regs, const char *minicore_path, const char *msg)
{
    struct crashlog   log;
    mm_segment_t      fs;
	struct vma_info   vma_info;
	int ret;

    ret = crashlog_init(&log, minicore_path);
    if (ret)
    {
        goto err_crashlog;
    }

    /* you can't write to files without this */
    fs = get_fs();
    set_fs(KERNEL_DS);
    /* you may write to log file now */

    if (msg)
    {
        crashlog_printk(&log, "Message: ");
        crashlog_printk(&log, msg);
        crashlog_printk(&log, "\n");
    }
    else
    {
        record_cause(&log, task, signr);
    }

	memset(&vma_info, 0, sizeof(vma_info));
	
#ifndef MINICORE_NO_MAPS
    record_maps(&log, (struct task_struct *)task, &vma_info);
#endif

    find_stacktrace(&log, task, regs, &vma_info);

    /* write to file */
    crashlog_dump(&log);

    /* clean up */
    set_fs(fs);
    crashlog_deinit(&log);

    return 0;

err_crashlog:
    return ret;
}


int minicore_backtrace(struct task_struct *task, const char *minicore_path,
	const char *msg)
{
    return dump_minicore_format(task, 0, NULL, minicore_path, msg);
}


/**
 * Called by signal handler when dumping of core is needed.
 */
void minicore(struct task_struct *task, int signr, struct pt_regs *regs)
{
    char minicore_path[CORENAME_MAX_SIZE + 1];

    /*
     * lock_kernel() because format_corename() is controlled by sysctl, which
     * uses lock_kernel()
     */
    lock_kernel();
    (void) format_corename(minicore_path, minicore_pattern, signr, NULL);
    unlock_kernel();

    if (dump_minicore_format(task, signr, regs, minicore_path, NULL))
    {
        printk("Process '%s' terminated, "
               "but no minicore could be generated.\n",
               task->comm);
        return;
    }
}


static int __init minicore_init(void)
{
    return 0;
}


static void __exit minicore_exit(void)
{
}


module_init(minicore_init);
module_exit(minicore_exit);


EXPORT_SYMBOL(minicore_backtrace);

