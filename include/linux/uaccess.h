#ifndef __LINUX_UACCESS_H__
#define __LINUX_UACCESS_H__

#include <linux/preempt.h>
#include <asm/uaccess.h>

static inline void pagefault_disable(void)
{
	inc_preempt_count();
	barrier();
}

static inline void pagefault_enable(void)
{
	barrier();
	dec_preempt_count();
	barrier();
	preempt_check_resched();
}

#ifndef ARCH_HAS_NOCACHE_UACCESS

static inline unsigned long __copy_from_user_inatomic_nocache(void *to,
				const void __user *from, unsigned long n)
{
	return __copy_from_user_inatomic(to, from, n);
}

static inline unsigned long __copy_from_user_nocache(void *to,
				const void __user *from, unsigned long n)
{
	return __copy_from_user(to, from, n);
}

#endif		

#define probe_kernel_address(addr, retval)		\
	({						\
		long ret;				\
		mm_segment_t old_fs = get_fs();		\
							\
		set_fs(KERNEL_DS);			\
		pagefault_disable();			\
		ret = __copy_from_user_inatomic(&(retval), (__force typeof(retval) __user *)(addr), sizeof(retval));		\
		pagefault_enable();			\
		set_fs(old_fs);				\
		ret;					\
	})

extern long probe_kernel_read(void *dst, const void *src, size_t size);
extern long __probe_kernel_read(void *dst, const void *src, size_t size);

extern long notrace probe_kernel_write(void *dst, const void *src, size_t size);
extern long notrace __probe_kernel_write(void *dst, const void *src, size_t size);

#endif		
