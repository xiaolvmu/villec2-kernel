/* thread_info.h: common low-level thread information accessors
 *
 * Copyright (C) 2002  David Howells (dhowells@redhat.com)
 * - Incorporating suggestions made by Linus Torvalds
 */

#ifndef _LINUX_THREAD_INFO_H
#define _LINUX_THREAD_INFO_H

#include <linux/types.h>

struct timespec;
struct compat_timespec;

struct restart_block {
	long (*fn)(struct restart_block *);
	union {
		
		struct {
			u32 __user *uaddr;
			u32 val;
			u32 flags;
			u32 bitset;
			u64 time;
			u32 __user *uaddr2;
		} futex;
		
		struct {
			clockid_t clockid;
			struct timespec __user *rmtp;
#ifdef CONFIG_COMPAT
			struct compat_timespec __user *compat_rmtp;
#endif
			u64 expires;
		} nanosleep;
		
		struct {
			struct pollfd __user *ufds;
			int nfds;
			int has_timeout;
			unsigned long tv_sec;
			unsigned long tv_nsec;
		} poll;
	};
};

extern long do_no_restart_syscall(struct restart_block *parm);

#include <linux/bitops.h>
#include <asm/thread_info.h>

#ifdef __KERNEL__


static inline void set_ti_thread_flag(struct thread_info *ti, int flag)
{
	set_bit(flag, (unsigned long *)&ti->flags);
}

static inline void clear_ti_thread_flag(struct thread_info *ti, int flag)
{
	clear_bit(flag, (unsigned long *)&ti->flags);
}

static inline int test_and_set_ti_thread_flag(struct thread_info *ti, int flag)
{
	return test_and_set_bit(flag, (unsigned long *)&ti->flags);
}

static inline int test_and_clear_ti_thread_flag(struct thread_info *ti, int flag)
{
	return test_and_clear_bit(flag, (unsigned long *)&ti->flags);
}

static inline int test_ti_thread_flag(struct thread_info *ti, int flag)
{
	return test_bit(flag, (unsigned long *)&ti->flags);
}

#define set_thread_flag(flag) \
	set_ti_thread_flag(current_thread_info(), flag)
#define clear_thread_flag(flag) \
	clear_ti_thread_flag(current_thread_info(), flag)
#define test_and_set_thread_flag(flag) \
	test_and_set_ti_thread_flag(current_thread_info(), flag)
#define test_and_clear_thread_flag(flag) \
	test_and_clear_ti_thread_flag(current_thread_info(), flag)
#define test_thread_flag(flag) \
	test_ti_thread_flag(current_thread_info(), flag)

#define set_need_resched()	set_thread_flag(TIF_NEED_RESCHED)
#define clear_need_resched()	clear_thread_flag(TIF_NEED_RESCHED)

#if defined TIF_RESTORE_SIGMASK && !defined HAVE_SET_RESTORE_SIGMASK
#define HAVE_SET_RESTORE_SIGMASK	1

static inline void set_restore_sigmask(void)
{
	set_thread_flag(TIF_RESTORE_SIGMASK);
	set_thread_flag(TIF_SIGPENDING);
}
#endif	

#endif	

#endif 
