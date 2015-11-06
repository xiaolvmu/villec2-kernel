
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sort.h>
#include <linux/slab.h>

static void u32_swap(void *a, void *b, int size)
{
	u32 t = *(u32 *)a;
	*(u32 *)a = *(u32 *)b;
	*(u32 *)b = t;
}

static void generic_swap(void *a, void *b, int size)
{
	char t;

	do {
		t = *(char *)a;
		*(char *)a++ = *(char *)b;
		*(char *)b++ = t;
	} while (--size > 0);
}


void sort(void *base, size_t num, size_t size,
	  int (*cmp_func)(const void *, const void *),
	  void (*swap_func)(void *, void *, int size))
{
	
	int i = (num/2 - 1) * size, n = num * size, c, r;

	if (!swap_func)
		swap_func = (size == 4 ? u32_swap : generic_swap);

	
	for ( ; i >= 0; i -= size) {
		for (r = i; r * 2 + size < n; r  = c) {
			c = r * 2 + size;
			if (c < n - size &&
					cmp_func(base + c, base + c + size) < 0)
				c += size;
			if (cmp_func(base + r, base + c) >= 0)
				break;
			swap_func(base + r, base + c, size);
		}
	}

	
	for (i = n - size; i > 0; i -= size) {
		swap_func(base, base + i, size);
		for (r = 0; r * 2 + size < i; r = c) {
			c = r * 2 + size;
			if (c < i - size &&
					cmp_func(base + c, base + c + size) < 0)
				c += size;
			if (cmp_func(base + r, base + c) >= 0)
				break;
			swap_func(base + r, base + c, size);
		}
	}
}

EXPORT_SYMBOL(sort);

#if 0

int cmpint(const void *a, const void *b)
{
	return *(int *)a - *(int *)b;
}

static int sort_test(void)
{
	int *a, i, r = 1;

	a = kmalloc(1000 * sizeof(int), GFP_KERNEL);
	BUG_ON(!a);

	printk("testing sort()\n");

	for (i = 0; i < 1000; i++) {
		r = (r * 725861) % 6599;
		a[i] = r;
	}

	sort(a, 1000, sizeof(int), cmpint, NULL);

	for (i = 0; i < 999; i++)
		if (a[i] > a[i+1]) {
			printk("sort() failed!\n");
			break;
		}

	kfree(a);

	return 0;
}

module_init(sort_test);
#endif
