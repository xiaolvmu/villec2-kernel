#include <linux/export.h>
#include <linux/mm.h>
#include <linux/uaccess.h>


long __weak probe_kernel_read(void *dst, const void *src, size_t size)
    __attribute__((alias("__probe_kernel_read")));

long __probe_kernel_read(void *dst, const void *src, size_t size)
{
	long ret;
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);
	pagefault_disable();
	ret = __copy_from_user_inatomic(dst,
			(__force const void __user *)src, size);
	pagefault_enable();
	set_fs(old_fs);

	return ret ? -EFAULT : 0;
}
EXPORT_SYMBOL_GPL(probe_kernel_read);

long __weak probe_kernel_write(void *dst, const void *src, size_t size)
    __attribute__((alias("__probe_kernel_write")));

long __probe_kernel_write(void *dst, const void *src, size_t size)
{
	long ret;
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);
	pagefault_disable();
	ret = __copy_to_user_inatomic((__force void __user *)dst, src, size);
	pagefault_enable();
	set_fs(old_fs);

	return ret ? -EFAULT : 0;
}
EXPORT_SYMBOL_GPL(probe_kernel_write);
