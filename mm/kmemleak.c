/*
 * mm/kmemleak.c
 *
 * Copyright (C) 2008 ARM Limited
 * Written by Catalin Marinas <catalin.marinas@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *
 * For more information on the algorithm and kmemleak usage, please see
 * Documentation/kmemleak.txt.
 *
 * Notes on locking
 * ----------------
 *
 * The following locks and mutexes are used by kmemleak:
 *
 * - kmemleak_lock (rwlock): protects the object_list modifications and
 *   accesses to the object_tree_root. The object_list is the main list
 *   holding the metadata (struct kmemleak_object) for the allocated memory
 *   blocks. The object_tree_root is a priority search tree used to look-up
 *   metadata based on a pointer to the corresponding memory block.  The
 *   kmemleak_object structures are added to the object_list and
 *   object_tree_root in the create_object() function called from the
 *   kmemleak_alloc() callback and removed in delete_object() called from the
 *   kmemleak_free() callback
 * - kmemleak_object.lock (spinlock): protects a kmemleak_object. Accesses to
 *   the metadata (e.g. count) are protected by this lock. Note that some
 *   members of this structure may be protected by other means (atomic or
 *   kmemleak_lock). This lock is also held when scanning the corresponding
 *   memory block to avoid the kernel freeing it via the kmemleak_free()
 *   callback. This is less heavyweight than holding a global lock like
 *   kmemleak_lock during scanning
 * - scan_mutex (mutex): ensures that only one thread may scan the memory for
 *   unreferenced objects at a time. The gray_list contains the objects which
 *   are already referenced or marked as false positives and need to be
 *   scanned. This list is only modified during a scanning episode when the
 *   scan_mutex is held. At the end of a scan, the gray_list is always empty.
 *   Note that the kmemleak_object.use_count is incremented when an object is
 *   added to the gray_list and therefore cannot be freed. This mutex also
 *   prevents multiple users of the "kmemleak" debugfs file together with
 *   modifications to the memory scanning parameters including the scan_thread
 *   pointer
 *
 * The kmemleak_object structures have a use_count incremented or decremented
 * using the get_object()/put_object() functions. When the use_count becomes
 * 0, this count can no longer be incremented and put_object() schedules the
 * kmemleak_object freeing via an RCU callback. All calls to the get_object()
 * function must be protected by rcu_read_lock() to avoid accessing a freed
 * structure.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/kthread.h>
#include <linux/prio_tree.h>
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/cpumask.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/rcupdate.h>
#include <linux/stacktrace.h>
#include <linux/cache.h>
#include <linux/percpu.h>
#include <linux/hardirq.h>
#include <linux/mmzone.h>
#include <linux/slab.h>
#include <linux/thread_info.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/nodemask.h>
#include <linux/mm.h>
#include <linux/workqueue.h>
#include <linux/crc32.h>

#include <asm/sections.h>
#include <asm/processor.h>
#include <linux/atomic.h>

#include <linux/kmemcheck.h>
#include <linux/kmemleak.h>
#include <linux/memory_hotplug.h>

#define MAX_TRACE		16	
#define MSECS_MIN_AGE		5000	
#define SECS_FIRST_SCAN		60	
#define SECS_SCAN_WAIT		600	
#define MAX_SCAN_SIZE		4096	

#define BYTES_PER_POINTER	sizeof(void *)

#define gfp_kmemleak_mask(gfp)	(((gfp) & (GFP_KERNEL | GFP_ATOMIC)) | \
				 __GFP_NORETRY | __GFP_NOMEMALLOC | \
				 __GFP_NOWARN)

struct kmemleak_scan_area {
	struct hlist_node node;
	unsigned long start;
	size_t size;
};

#define KMEMLEAK_GREY	0
#define KMEMLEAK_BLACK	-1

struct kmemleak_object {
	spinlock_t lock;
	unsigned long flags;		
	struct list_head object_list;
	struct list_head gray_list;
	struct prio_tree_node tree_node;
	struct rcu_head rcu;		
	
	atomic_t use_count;
	unsigned long pointer;
	size_t size;
	
	int min_count;
	
	int count;
	
	u32 checksum;
	
	struct hlist_head area_list;
	unsigned long trace[MAX_TRACE];
	unsigned int trace_len;
	unsigned long jiffies;		
	pid_t pid;			
	char comm[TASK_COMM_LEN];	
	unsigned long long ktime;       
	unsigned long nanosec_rem;      
};

#define OBJECT_ALLOCATED	(1 << 0)
#define OBJECT_REPORTED		(1 << 1)
#define OBJECT_NO_SCAN		(1 << 2)

#define HEX_ROW_SIZE		16
#define HEX_GROUP_SIZE		1
#define HEX_ASCII		1
#define HEX_MAX_LINES		2

static LIST_HEAD(object_list);
static LIST_HEAD(gray_list);
static struct prio_tree_root object_tree_root;
static DEFINE_RWLOCK(kmemleak_lock);

static struct kmem_cache *object_cache;
static struct kmem_cache *scan_area_cache;

static atomic_t kmemleak_enabled = ATOMIC_INIT(0);
static atomic_t kmemleak_initialized = ATOMIC_INIT(0);
static atomic_t kmemleak_early_log = ATOMIC_INIT(1);
static atomic_t kmemleak_warning = ATOMIC_INIT(0);
static atomic_t kmemleak_error = ATOMIC_INIT(0);

static unsigned long min_addr = ULONG_MAX;
static unsigned long max_addr;

static struct task_struct *scan_thread;
static unsigned long jiffies_min_age;
static unsigned long jiffies_last_scan;
static signed long jiffies_scan_wait;
static int kmemleak_stack_scan = 1;
static DEFINE_MUTEX(scan_mutex);
static int kmemleak_skip_disable;



enum {
	KMEMLEAK_ALLOC,
	KMEMLEAK_ALLOC_PERCPU,
	KMEMLEAK_FREE,
	KMEMLEAK_FREE_PART,
	KMEMLEAK_FREE_PERCPU,
	KMEMLEAK_NOT_LEAK,
	KMEMLEAK_IGNORE,
	KMEMLEAK_SCAN_AREA,
	KMEMLEAK_NO_SCAN
};

struct early_log {
	int op_type;			
	const void *ptr;		
	size_t size;			
	int min_count;			
	unsigned long trace[MAX_TRACE];	
	unsigned int trace_len;		
};

static struct early_log
	early_log[CONFIG_DEBUG_KMEMLEAK_EARLY_LOG_SIZE] __initdata;
static int crt_early_log __initdata;

static void kmemleak_disable(void);

#define kmemleak_warn(x...)	do {		\
	pr_warning(x);				\
	dump_stack();				\
	atomic_set(&kmemleak_warning, 1);	\
} while (0)

#define kmemleak_stop(x...)	do {	\
	kmemleak_warn(x);		\
	kmemleak_disable();		\
} while (0)

static void hex_dump_object(struct seq_file *seq,
			    struct kmemleak_object *object)
{
	const u8 *ptr = (const u8 *)object->pointer;
	int i, len, remaining;
	unsigned char linebuf[HEX_ROW_SIZE * 5];

	
	remaining = len =
		min(object->size, (size_t)(HEX_MAX_LINES * HEX_ROW_SIZE));

	seq_printf(seq, "  hex dump (first %d bytes):\n", len);
	for (i = 0; i < len; i += HEX_ROW_SIZE) {
		int linelen = min(remaining, HEX_ROW_SIZE);

		remaining -= HEX_ROW_SIZE;
		hex_dump_to_buffer(ptr + i, linelen, HEX_ROW_SIZE,
				   HEX_GROUP_SIZE, linebuf, sizeof(linebuf),
				   HEX_ASCII);
		seq_printf(seq, "    %s\n", linebuf);
	}
}

static bool color_white(const struct kmemleak_object *object)
{
	return object->count != KMEMLEAK_BLACK &&
		object->count < object->min_count;
}

static bool color_gray(const struct kmemleak_object *object)
{
	return object->min_count != KMEMLEAK_BLACK &&
		object->count >= object->min_count;
}

static bool unreferenced_object(struct kmemleak_object *object)
{
	return (color_white(object) && object->flags & OBJECT_ALLOCATED) &&
		time_before_eq(object->jiffies + jiffies_min_age,
			       jiffies_last_scan);
}

static void print_unreferenced(struct seq_file *seq,
			       struct kmemleak_object *object)
{
	int i;
	unsigned int msecs_age = jiffies_to_msecs(jiffies - object->jiffies);

	seq_printf(seq, "unreferenced object 0x%08lx (size %zu):\n",
		   object->pointer, object->size);
	seq_printf(seq, "  comm \"%s\", pid %d, jiffies %lu (age %d.%03ds) [%5lu.%06lu]\n",
		   object->comm, object->pid, object->jiffies,
		   msecs_age / 1000, msecs_age % 1000,
		   (unsigned long) object->ktime, object->nanosec_rem / 1000);
	hex_dump_object(seq, object);
	seq_printf(seq, "  backtrace:\n");

	for (i = 0; i < object->trace_len; i++) {
		void *ptr = (void *)object->trace[i];
		seq_printf(seq, "    [<%p>] %pS\n", ptr, ptr);
	}
}

static void dump_object_info(struct kmemleak_object *object)
{
	struct stack_trace trace;

	trace.nr_entries = object->trace_len;
	trace.entries = object->trace;

	pr_notice("Object 0x%08lx (size %zu):\n",
		  object->tree_node.start, object->size);
	pr_notice("  comm \"%s\", pid %d, jiffies %lu\n",
		  object->comm, object->pid, object->jiffies);
	pr_notice("  min_count = %d\n", object->min_count);
	pr_notice("  count = %d\n", object->count);
	pr_notice("  flags = 0x%lx\n", object->flags);
	pr_notice("  checksum = %d\n", object->checksum);
	pr_notice("  backtrace:\n");
	print_stack_trace(&trace, 4);
}

static struct kmemleak_object *lookup_object(unsigned long ptr, int alias)
{
	struct prio_tree_node *node;
	struct prio_tree_iter iter;
	struct kmemleak_object *object;

	prio_tree_iter_init(&iter, &object_tree_root, ptr, ptr);
	node = prio_tree_next(&iter);
	if (node) {
		object = prio_tree_entry(node, struct kmemleak_object,
					 tree_node);
		if (!alias && object->pointer != ptr) {
			kmemleak_warn("Found object by alias at 0x%08lx\n",
				      ptr);
			dump_object_info(object);
			object = NULL;
		}
	} else
		object = NULL;

	return object;
}

static int get_object(struct kmemleak_object *object)
{
	return atomic_inc_not_zero(&object->use_count);
}

static void free_object_rcu(struct rcu_head *rcu)
{
	struct hlist_node *elem, *tmp;
	struct kmemleak_scan_area *area;
	struct kmemleak_object *object =
		container_of(rcu, struct kmemleak_object, rcu);

	hlist_for_each_entry_safe(area, elem, tmp, &object->area_list, node) {
		hlist_del(elem);
		kmem_cache_free(scan_area_cache, area);
	}
	kmem_cache_free(object_cache, object);
}

static void put_object(struct kmemleak_object *object)
{
	if (!atomic_dec_and_test(&object->use_count))
		return;

	
	WARN_ON(object->flags & OBJECT_ALLOCATED);

	call_rcu(&object->rcu, free_object_rcu);
}

static struct kmemleak_object *find_and_get_object(unsigned long ptr, int alias)
{
	unsigned long flags;
	struct kmemleak_object *object = NULL;

	rcu_read_lock();
	read_lock_irqsave(&kmemleak_lock, flags);
	if (ptr >= min_addr && ptr < max_addr)
		object = lookup_object(ptr, alias);
	read_unlock_irqrestore(&kmemleak_lock, flags);

	
	if (object && !get_object(object))
		object = NULL;
	rcu_read_unlock();

	return object;
}

static int __save_stack_trace(unsigned long *trace)
{
	struct stack_trace stack_trace;

	stack_trace.max_entries = MAX_TRACE;
	stack_trace.nr_entries = 0;
	stack_trace.entries = trace;
	stack_trace.skip = 2;
	save_stack_trace(&stack_trace);

	return stack_trace.nr_entries;
}

static struct kmemleak_object *create_object(unsigned long ptr, size_t size,
					     int min_count, gfp_t gfp)
{
	unsigned long flags;
	struct kmemleak_object *object;
	struct prio_tree_node *node;

	object = kmem_cache_alloc(object_cache, gfp_kmemleak_mask(gfp));
	if (!object) {
		pr_warning("Cannot allocate a kmemleak_object structure\n");
		kmemleak_disable();
		return NULL;
	}

	INIT_LIST_HEAD(&object->object_list);
	INIT_LIST_HEAD(&object->gray_list);
	INIT_HLIST_HEAD(&object->area_list);
	spin_lock_init(&object->lock);
	atomic_set(&object->use_count, 1);
	object->flags = OBJECT_ALLOCATED;
	object->pointer = ptr;
	object->size = size;
	object->min_count = min_count;
	object->count = 0;			
	object->jiffies = jiffies;
	object->checksum = 0;
	object->ktime = cpu_clock(UINT_MAX);
	object->nanosec_rem = do_div(object->ktime, 1000000000);

	
	if (in_irq()) {
		object->pid = 0;
		strncpy(object->comm, "hardirq", sizeof(object->comm));
	} else if (in_softirq()) {
		object->pid = 0;
		strncpy(object->comm, "softirq", sizeof(object->comm));
	} else {
		object->pid = current->pid;
		strncpy(object->comm, current->comm, sizeof(object->comm));
	}

	
	object->trace_len = __save_stack_trace(object->trace);

	INIT_PRIO_TREE_NODE(&object->tree_node);
	object->tree_node.start = ptr;
	object->tree_node.last = ptr + size - 1;

	write_lock_irqsave(&kmemleak_lock, flags);

	min_addr = min(min_addr, ptr);
	max_addr = max(max_addr, ptr + size);
	node = prio_tree_insert(&object_tree_root, &object->tree_node);
	if (node != &object->tree_node) {
		kmemleak_stop("Cannot insert 0x%lx into the object search tree "
			      "(already existing)\n", ptr);
		object = lookup_object(ptr, 1);
		spin_lock(&object->lock);
		dump_object_info(object);
		spin_unlock(&object->lock);

		goto out;
	}
	list_add_tail_rcu(&object->object_list, &object_list);
out:
	write_unlock_irqrestore(&kmemleak_lock, flags);
	return object;
}

static void __delete_object(struct kmemleak_object *object)
{
	unsigned long flags;

	write_lock_irqsave(&kmemleak_lock, flags);
	prio_tree_remove(&object_tree_root, &object->tree_node);
	list_del_rcu(&object->object_list);
	write_unlock_irqrestore(&kmemleak_lock, flags);

	WARN_ON(!(object->flags & OBJECT_ALLOCATED));
	WARN_ON(atomic_read(&object->use_count) < 2);

	spin_lock_irqsave(&object->lock, flags);
	object->flags &= ~OBJECT_ALLOCATED;
	spin_unlock_irqrestore(&object->lock, flags);
	put_object(object);
}

static void delete_object_full(unsigned long ptr)
{
	struct kmemleak_object *object;

	object = find_and_get_object(ptr, 0);
	if (!object) {
#ifdef DEBUG
		kmemleak_warn("Freeing unknown object at 0x%08lx\n",
			      ptr);
#endif
		return;
	}
	__delete_object(object);
	put_object(object);
}

static void delete_object_part(unsigned long ptr, size_t size)
{
	struct kmemleak_object *object;
	unsigned long start, end;

	object = find_and_get_object(ptr, 1);
	if (!object) {
#ifdef DEBUG
		kmemleak_warn("Partially freeing unknown object at 0x%08lx "
			      "(size %zu)\n", ptr, size);
#endif
		return;
	}
	__delete_object(object);

	start = object->pointer;
	end = object->pointer + object->size;
	if (ptr > start)
		create_object(start, ptr - start, object->min_count,
			      GFP_KERNEL);
	if (ptr + size < end)
		create_object(ptr + size, end - ptr - size, object->min_count,
			      GFP_KERNEL);

	put_object(object);
}

static void __paint_it(struct kmemleak_object *object, int color)
{
	object->min_count = color;
	if (color == KMEMLEAK_BLACK)
		object->flags |= OBJECT_NO_SCAN;
}

static void paint_it(struct kmemleak_object *object, int color)
{
	unsigned long flags;

	spin_lock_irqsave(&object->lock, flags);
	__paint_it(object, color);
	spin_unlock_irqrestore(&object->lock, flags);
}

static void paint_ptr(unsigned long ptr, int color)
{
	struct kmemleak_object *object;

	object = find_and_get_object(ptr, 0);
	if (!object) {
		kmemleak_warn("Trying to color unknown object "
			      "at 0x%08lx as %s\n", ptr,
			      (color == KMEMLEAK_GREY) ? "Grey" :
			      (color == KMEMLEAK_BLACK) ? "Black" : "Unknown");
		return;
	}
	paint_it(object, color);
	put_object(object);
}

static void make_gray_object(unsigned long ptr)
{
	paint_ptr(ptr, KMEMLEAK_GREY);
}

static void make_black_object(unsigned long ptr)
{
	paint_ptr(ptr, KMEMLEAK_BLACK);
}

static void add_scan_area(unsigned long ptr, size_t size, gfp_t gfp)
{
	unsigned long flags;
	struct kmemleak_object *object;
	struct kmemleak_scan_area *area;

	object = find_and_get_object(ptr, 1);
	if (!object) {
		kmemleak_warn("Adding scan area to unknown object at 0x%08lx\n",
			      ptr);
		return;
	}

	area = kmem_cache_alloc(scan_area_cache, gfp_kmemleak_mask(gfp));
	if (!area) {
		pr_warning("Cannot allocate a scan area\n");
		goto out;
	}

	spin_lock_irqsave(&object->lock, flags);
	if (ptr + size > object->pointer + object->size) {
		kmemleak_warn("Scan area larger than object 0x%08lx\n", ptr);
		dump_object_info(object);
		kmem_cache_free(scan_area_cache, area);
		goto out_unlock;
	}

	INIT_HLIST_NODE(&area->node);
	area->start = ptr;
	area->size = size;

	hlist_add_head(&area->node, &object->area_list);
out_unlock:
	spin_unlock_irqrestore(&object->lock, flags);
out:
	put_object(object);
}

static void object_no_scan(unsigned long ptr)
{
	unsigned long flags;
	struct kmemleak_object *object;

	object = find_and_get_object(ptr, 0);
	if (!object) {
		kmemleak_warn("Not scanning unknown object at 0x%08lx\n", ptr);
		return;
	}

	spin_lock_irqsave(&object->lock, flags);
	object->flags |= OBJECT_NO_SCAN;
	spin_unlock_irqrestore(&object->lock, flags);
	put_object(object);
}

static void __init log_early(int op_type, const void *ptr, size_t size,
			     int min_count)
{
	unsigned long flags;
	struct early_log *log;

	if (atomic_read(&kmemleak_error)) {
		
		crt_early_log++;
		return;
	}

	if (crt_early_log >= ARRAY_SIZE(early_log)) {
		kmemleak_disable();
		return;
	}

	local_irq_save(flags);
	log = &early_log[crt_early_log];
	log->op_type = op_type;
	log->ptr = ptr;
	log->size = size;
	log->min_count = min_count;
	log->trace_len = __save_stack_trace(log->trace);
	crt_early_log++;
	local_irq_restore(flags);
}

static void early_alloc(struct early_log *log)
{
	struct kmemleak_object *object;
	unsigned long flags;
	int i;

	if (!atomic_read(&kmemleak_enabled) || !log->ptr || IS_ERR(log->ptr))
		return;

	rcu_read_lock();
	object = create_object((unsigned long)log->ptr, log->size,
			       log->min_count, GFP_ATOMIC);
	if (!object)
		goto out;
	spin_lock_irqsave(&object->lock, flags);
	for (i = 0; i < log->trace_len; i++)
		object->trace[i] = log->trace[i];
	object->trace_len = log->trace_len;
	spin_unlock_irqrestore(&object->lock, flags);
out:
	rcu_read_unlock();
}

static void early_alloc_percpu(struct early_log *log)
{
	unsigned int cpu;
	const void __percpu *ptr = log->ptr;

	for_each_possible_cpu(cpu) {
		log->ptr = per_cpu_ptr(ptr, cpu);
		early_alloc(log);
	}
}

void __ref kmemleak_alloc(const void *ptr, size_t size, int min_count,
			  gfp_t gfp)
{
	pr_debug("%s(0x%p, %zu, %d)\n", __func__, ptr, size, min_count);

	if (atomic_read(&kmemleak_enabled) && ptr && !IS_ERR(ptr))
		create_object((unsigned long)ptr, size, min_count, gfp);
	else if (atomic_read(&kmemleak_early_log))
		log_early(KMEMLEAK_ALLOC, ptr, size, min_count);
}
EXPORT_SYMBOL_GPL(kmemleak_alloc);

void __ref kmemleak_alloc_percpu(const void __percpu *ptr, size_t size)
{
	unsigned int cpu;

	pr_debug("%s(0x%p, %zu)\n", __func__, ptr, size);

	if (atomic_read(&kmemleak_enabled) && ptr && !IS_ERR(ptr))
		for_each_possible_cpu(cpu)
			create_object((unsigned long)per_cpu_ptr(ptr, cpu),
				      size, 0, GFP_KERNEL);
	else if (atomic_read(&kmemleak_early_log))
		log_early(KMEMLEAK_ALLOC_PERCPU, ptr, size, 0);
}
EXPORT_SYMBOL_GPL(kmemleak_alloc_percpu);

void __ref kmemleak_free(const void *ptr)
{
	pr_debug("%s(0x%p)\n", __func__, ptr);

	if (atomic_read(&kmemleak_enabled) && ptr && !IS_ERR(ptr))
		delete_object_full((unsigned long)ptr);
	else if (atomic_read(&kmemleak_early_log))
		log_early(KMEMLEAK_FREE, ptr, 0, 0);
}
EXPORT_SYMBOL_GPL(kmemleak_free);

void __ref kmemleak_free_part(const void *ptr, size_t size)
{
	pr_debug("%s(0x%p)\n", __func__, ptr);

	if (atomic_read(&kmemleak_enabled) && ptr && !IS_ERR(ptr))
		delete_object_part((unsigned long)ptr, size);
	else if (atomic_read(&kmemleak_early_log))
		log_early(KMEMLEAK_FREE_PART, ptr, size, 0);
}
EXPORT_SYMBOL_GPL(kmemleak_free_part);

void __ref kmemleak_free_percpu(const void __percpu *ptr)
{
	unsigned int cpu;

	pr_debug("%s(0x%p)\n", __func__, ptr);

	if (atomic_read(&kmemleak_enabled) && ptr && !IS_ERR(ptr))
		for_each_possible_cpu(cpu)
			delete_object_full((unsigned long)per_cpu_ptr(ptr,
								      cpu));
	else if (atomic_read(&kmemleak_early_log))
		log_early(KMEMLEAK_FREE_PERCPU, ptr, 0, 0);
}
EXPORT_SYMBOL_GPL(kmemleak_free_percpu);

void __ref kmemleak_not_leak(const void *ptr)
{
	pr_debug("%s(0x%p)\n", __func__, ptr);

	if (atomic_read(&kmemleak_enabled) && ptr && !IS_ERR(ptr))
		make_gray_object((unsigned long)ptr);
	else if (atomic_read(&kmemleak_early_log))
		log_early(KMEMLEAK_NOT_LEAK, ptr, 0, 0);
}
EXPORT_SYMBOL(kmemleak_not_leak);

void __ref kmemleak_ignore(const void *ptr)
{
	pr_debug("%s(0x%p)\n", __func__, ptr);

	if (atomic_read(&kmemleak_enabled) && ptr && !IS_ERR(ptr))
		make_black_object((unsigned long)ptr);
	else if (atomic_read(&kmemleak_early_log))
		log_early(KMEMLEAK_IGNORE, ptr, 0, 0);
}
EXPORT_SYMBOL(kmemleak_ignore);

void __ref kmemleak_scan_area(const void *ptr, size_t size, gfp_t gfp)
{
	pr_debug("%s(0x%p)\n", __func__, ptr);

	if (atomic_read(&kmemleak_enabled) && ptr && size && !IS_ERR(ptr))
		add_scan_area((unsigned long)ptr, size, gfp);
	else if (atomic_read(&kmemleak_early_log))
		log_early(KMEMLEAK_SCAN_AREA, ptr, size, 0);
}
EXPORT_SYMBOL(kmemleak_scan_area);

void __ref kmemleak_no_scan(const void *ptr)
{
	pr_debug("%s(0x%p)\n", __func__, ptr);

	if (atomic_read(&kmemleak_enabled) && ptr && !IS_ERR(ptr))
		object_no_scan((unsigned long)ptr);
	else if (atomic_read(&kmemleak_early_log))
		log_early(KMEMLEAK_NO_SCAN, ptr, 0, 0);
}
EXPORT_SYMBOL(kmemleak_no_scan);

static bool update_checksum(struct kmemleak_object *object)
{
	u32 old_csum = object->checksum;

	if (!kmemcheck_is_obj_initialized(object->pointer, object->size))
		return false;

	object->checksum = crc32(0, (void *)object->pointer, object->size);
	return object->checksum != old_csum;
}

static int scan_should_stop(void)
{
	if (!atomic_read(&kmemleak_enabled))
		return 1;

	if (current->mm)
		return signal_pending(current);
	else
		return kthread_should_stop();

	return 0;
}

static void scan_block(void *_start, void *_end,
		       struct kmemleak_object *scanned, int allow_resched)
{
	unsigned long *ptr;
	unsigned long *start = PTR_ALIGN(_start, BYTES_PER_POINTER);
	unsigned long *end = _end - (BYTES_PER_POINTER - 1);

	for (ptr = start; ptr < end; ptr++) {
		struct kmemleak_object *object;
		unsigned long flags;
		unsigned long pointer;

		if (allow_resched)
			cond_resched();
		if (scan_should_stop())
			break;

		
		if (!kmemcheck_is_obj_initialized((unsigned long)ptr,
						  BYTES_PER_POINTER))
			continue;

		pointer = *ptr;

		object = find_and_get_object(pointer, 1);
		if (!object)
			continue;
		if (object == scanned) {
			
			put_object(object);
			continue;
		}

		spin_lock_irqsave_nested(&object->lock, flags,
					 SINGLE_DEPTH_NESTING);
		if (!color_white(object)) {
			
			spin_unlock_irqrestore(&object->lock, flags);
			put_object(object);
			continue;
		}

		object->count++;
		if (color_gray(object)) {
			list_add_tail(&object->gray_list, &gray_list);
			spin_unlock_irqrestore(&object->lock, flags);
			continue;
		}

		spin_unlock_irqrestore(&object->lock, flags);
		put_object(object);
	}
}

static void scan_object(struct kmemleak_object *object)
{
	struct kmemleak_scan_area *area;
	struct hlist_node *elem;
	unsigned long flags;

	spin_lock_irqsave(&object->lock, flags);
	if (object->flags & OBJECT_NO_SCAN)
		goto out;
	if (!(object->flags & OBJECT_ALLOCATED))
		
		goto out;
	if (hlist_empty(&object->area_list)) {
		void *start = (void *)object->pointer;
		void *end = (void *)(object->pointer + object->size);

		while (start < end && (object->flags & OBJECT_ALLOCATED) &&
		       !(object->flags & OBJECT_NO_SCAN)) {
			scan_block(start, min(start + MAX_SCAN_SIZE, end),
				   object, 0);
			start += MAX_SCAN_SIZE;

			spin_unlock_irqrestore(&object->lock, flags);
			cond_resched();
			spin_lock_irqsave(&object->lock, flags);
		}
	} else
		hlist_for_each_entry(area, elem, &object->area_list, node)
			scan_block((void *)area->start,
				   (void *)(area->start + area->size),
				   object, 0);
out:
	spin_unlock_irqrestore(&object->lock, flags);
}

static void scan_gray_list(void)
{
	struct kmemleak_object *object, *tmp;

	object = list_entry(gray_list.next, typeof(*object), gray_list);
	while (&object->gray_list != &gray_list) {
		cond_resched();

		
		if (!scan_should_stop())
			scan_object(object);

		tmp = list_entry(object->gray_list.next, typeof(*object),
				 gray_list);

		
		list_del(&object->gray_list);
		put_object(object);

		object = tmp;
	}
	WARN_ON(!list_empty(&gray_list));
}

static unsigned int kmalloc_cnt[1024];
static void kmemleak_scan(void)
{
	unsigned long flags;
	struct kmemleak_object *object;
	int i;
	int new_leaks = 0;
	int j;
	unsigned int leak_max, print_bool = 0;

	jiffies_last_scan = jiffies;

	
	rcu_read_lock();
	memset(kmalloc_cnt, 0, sizeof(unsigned int)*1024);
	list_for_each_entry_rcu(object, &object_list, object_list) {
		spin_lock_irqsave(&object->lock, flags);
#ifdef DEBUG
		if (atomic_read(&object->use_count) > 1) {
			pr_debug("object->use_count = %d\n",
				 atomic_read(&object->use_count));
			dump_object_info(object);
		}
#endif
		
		object->count = 0;
		if (color_gray(object) && get_object(object))
			list_add_tail(&object->gray_list, &gray_list);

		if (object->size > 4096) {
			if (object->pid == 0 || object->pid == 1)
				kmalloc_cnt[1023] += object->size;
			else
				kmalloc_cnt[object->pid % 1023] += object->size;
		}
		spin_unlock_irqrestore(&object->lock, flags);
	}
	rcu_read_unlock();
	i = 0;
	leak_max = 0;
	while (i < 1024) {
		for (j = 0; j < 10 && i < 1024; j++, i++) {
			if (kmalloc_cnt[i] != 0) {
				if (print_bool == 0) {
					print_bool = 1;
					pr_info("KMleak: ");
				}
				pr_info(", %d[%u]", i, kmalloc_cnt[i]);
				if (leak_max < kmalloc_cnt[i])
					leak_max = kmalloc_cnt[i];
			}
		}
		if (print_bool) {
			pr_info("\n");
			print_bool = 0;
		}
	}
	pr_info("KMleak: MAX leak alloc times: %u\n", leak_max);

	show_mem(SHOW_MEM_FILTER_NODES);

	
	scan_block(_sdata, _edata, NULL, 1);
	scan_block(__bss_start, __bss_stop, NULL, 1);

#ifdef CONFIG_SMP
	
	for_each_possible_cpu(i)
		scan_block(__per_cpu_start + per_cpu_offset(i),
			   __per_cpu_end + per_cpu_offset(i), NULL, 1);
#endif

	lock_memory_hotplug();
	for_each_online_node(i) {
		pg_data_t *pgdat = NODE_DATA(i);
		unsigned long start_pfn = pgdat->node_start_pfn;
		unsigned long end_pfn = start_pfn + pgdat->node_spanned_pages;
		unsigned long pfn;

		for (pfn = start_pfn; pfn < end_pfn; pfn++) {
			struct page *page;

			if (!pfn_valid(pfn))
				continue;
			page = pfn_to_page(pfn);
			
			if (page_count(page) == 0)
				continue;
			scan_block(page, page + 1, NULL, 1);
		}
	}
	unlock_memory_hotplug();

	if (kmemleak_stack_scan) {
		struct task_struct *p, *g;

		read_lock(&tasklist_lock);
		do_each_thread(g, p) {
			scan_block(task_stack_page(p), task_stack_page(p) +
				   THREAD_SIZE, NULL, 0);
		} while_each_thread(g, p);
		read_unlock(&tasklist_lock);
	}

	scan_gray_list();

	rcu_read_lock();
	list_for_each_entry_rcu(object, &object_list, object_list) {
		spin_lock_irqsave(&object->lock, flags);
		if (color_white(object) && (object->flags & OBJECT_ALLOCATED)
		    && update_checksum(object) && get_object(object)) {
			
			object->count = object->min_count;
			list_add_tail(&object->gray_list, &gray_list);
		}
		spin_unlock_irqrestore(&object->lock, flags);
	}
	rcu_read_unlock();

	scan_gray_list();

	if (scan_should_stop())
		return;

	rcu_read_lock();
	list_for_each_entry_rcu(object, &object_list, object_list) {
		spin_lock_irqsave(&object->lock, flags);
		if (unreferenced_object(object) &&
		    !(object->flags & OBJECT_REPORTED)) {
			object->flags |= OBJECT_REPORTED;
			new_leaks++;
		}
		spin_unlock_irqrestore(&object->lock, flags);
	}
	rcu_read_unlock();

	if (new_leaks)
#ifdef CONFIG_PROC_FS
		pr_info("%d new suspected memory leaks (see "
			"/proc/kmemleak)\n", new_leaks);
#else
		pr_info("%d new suspected memory leaks (see "
			"/sys/kernel/debug/kmemleak)\n", new_leaks);
#endif

}

static int kmemleak_scan_thread(void *arg)
{
	static int first_run = 1;

	pr_info("Automatic memory scanning thread started\n");
	set_user_nice(current, 10);

	if (first_run) {
		first_run = 0;
		ssleep(SECS_FIRST_SCAN);
	}

	while (!kthread_should_stop()) {
		signed long timeout = jiffies_scan_wait;

		mutex_lock(&scan_mutex);
		kmemleak_scan();
		mutex_unlock(&scan_mutex);

		
		while (timeout && !kthread_should_stop())
			timeout = schedule_timeout_interruptible(timeout);
	}

	pr_info("Automatic memory scanning thread ended\n");

	return 0;
}

static void start_scan_thread(void)
{
	if (scan_thread)
		return;
	scan_thread = kthread_run(kmemleak_scan_thread, NULL, "kmemleak");
	if (IS_ERR(scan_thread)) {
		pr_warning("Failed to create the scan thread\n");
		scan_thread = NULL;
	}
}

static void stop_scan_thread(void)
{
	if (scan_thread) {
		kthread_stop(scan_thread);
		scan_thread = NULL;
	}
}

static void *kmemleak_seq_start(struct seq_file *seq, loff_t *pos)
{
	struct kmemleak_object *object;
	loff_t n = *pos;
	int err;

	err = mutex_lock_interruptible(&scan_mutex);
	if (err < 0)
		return ERR_PTR(err);

	rcu_read_lock();
	list_for_each_entry_rcu(object, &object_list, object_list) {
		if (n-- > 0)
			continue;
		if (get_object(object))
			goto out;
	}
	object = NULL;
out:
	return object;
}

static void *kmemleak_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct kmemleak_object *prev_obj = v;
	struct kmemleak_object *next_obj = NULL;
	struct list_head *n = &prev_obj->object_list;

	++(*pos);

	list_for_each_continue_rcu(n, &object_list) {
		struct kmemleak_object *obj =
			list_entry(n, struct kmemleak_object, object_list);
		if (get_object(obj)) {
			next_obj = obj;
			break;
		}
	}

	put_object(prev_obj);
	return next_obj;
}

static void kmemleak_seq_stop(struct seq_file *seq, void *v)
{
	if (!IS_ERR(v)) {
		rcu_read_unlock();
		mutex_unlock(&scan_mutex);
		if (v)
			put_object(v);
	}
}

static int kmemleak_seq_show(struct seq_file *seq, void *v)
{
	struct kmemleak_object *object = v;
	unsigned long flags;

	spin_lock_irqsave(&object->lock, flags);
	if ((object->flags & OBJECT_REPORTED) && unreferenced_object(object))
		print_unreferenced(seq, object);
	spin_unlock_irqrestore(&object->lock, flags);
	return 0;
}

static const struct seq_operations kmemleak_seq_ops = {
	.start = kmemleak_seq_start,
	.next  = kmemleak_seq_next,
	.stop  = kmemleak_seq_stop,
	.show  = kmemleak_seq_show,
};

static int kmemleak_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &kmemleak_seq_ops);
}

static int kmemleak_release(struct inode *inode, struct file *file)
{
	return seq_release(inode, file);
}

static int dump_str_object_info(const char *str)
{
	unsigned long flags;
	struct kmemleak_object *object;
	unsigned long addr;

	addr= simple_strtoul(str, NULL, 0);
	object = find_and_get_object(addr, 0);
	if (!object) {
		pr_info("Unknown object at 0x%08lx\n", addr);
		return -EINVAL;
	}

	spin_lock_irqsave(&object->lock, flags);
	dump_object_info(object);
	spin_unlock_irqrestore(&object->lock, flags);

	put_object(object);
	return 0;
}

static void kmemleak_clear(void)
{
	struct kmemleak_object *object;
	unsigned long flags;

	rcu_read_lock();
	list_for_each_entry_rcu(object, &object_list, object_list) {
		spin_lock_irqsave(&object->lock, flags);
		if ((object->flags & OBJECT_REPORTED) &&
		    unreferenced_object(object))
			__paint_it(object, KMEMLEAK_GREY);
		spin_unlock_irqrestore(&object->lock, flags);
	}
	rcu_read_unlock();
}

/*
 * File write operation to configure kmemleak at run-time. The following
 * commands can be written to the /sys/kernel/debug/kmemleak file:
 *   off	- disable kmemleak (irreversible)
 *   stack=on	- enable the task stacks scanning
 *   stack=off	- disable the tasks stacks scanning
 *   scan=on	- start the automatic memory scanning thread
 *   scan=off	- stop the automatic memory scanning thread
 *   scan=...	- set the automatic memory scanning period in seconds (0 to
 *		  disable it)
 *   scan	- trigger a memory scan
 *   clear	- mark all current reported unreferenced kmemleak objects as
 *		  grey to ignore printing them
 *   dump=...	- dump information about the object found at the given address
 */
static ssize_t kmemleak_write(struct file *file, const char __user *user_buf,
			      size_t size, loff_t *ppos)
{
	char buf[64];
	int buf_size;
	int ret;

	if (!atomic_read(&kmemleak_enabled))
		return -EBUSY;

	buf_size = min(size, (sizeof(buf) - 1));
	if (strncpy_from_user(buf, user_buf, buf_size) < 0)
		return -EFAULT;
	buf[buf_size] = 0;

	ret = mutex_lock_interruptible(&scan_mutex);
	if (ret < 0)
		return ret;

	if (strncmp(buf, "off", 3) == 0)
		kmemleak_disable();
	else if (strncmp(buf, "stack=on", 8) == 0)
		kmemleak_stack_scan = 1;
	else if (strncmp(buf, "stack=off", 9) == 0)
		kmemleak_stack_scan = 0;
	else if (strncmp(buf, "scan=on", 7) == 0)
		start_scan_thread();
	else if (strncmp(buf, "scan=off", 8) == 0)
		stop_scan_thread();
	else if (strncmp(buf, "scan=", 5) == 0) {
		unsigned long secs;

		ret = strict_strtoul(buf + 5, 0, &secs);
		if (ret < 0)
			goto out;
		stop_scan_thread();
		if (secs) {
			jiffies_scan_wait = msecs_to_jiffies(secs * 1000);
			start_scan_thread();
		}
	} else if (strncmp(buf, "scan", 4) == 0)
		kmemleak_scan();
	else if (strncmp(buf, "clear", 5) == 0)
		kmemleak_clear();
	else if (strncmp(buf, "dump=", 5) == 0)
		ret = dump_str_object_info(buf + 5);
	else
		ret = -EINVAL;

out:
	mutex_unlock(&scan_mutex);
	if (ret < 0)
		return ret;

	
	*ppos += size;
	return size;
}

static const struct file_operations kmemleak_fops = {
	.owner		= THIS_MODULE,
	.open		= kmemleak_open,
	.read		= seq_read,
	.write		= kmemleak_write,
	.llseek		= seq_lseek,
	.release	= kmemleak_release,
};

static void kmemleak_do_cleanup(struct work_struct *work)
{
	struct kmemleak_object *object;
	bool cleanup = scan_thread == NULL;

	mutex_lock(&scan_mutex);
	stop_scan_thread();

	if (cleanup) {
		rcu_read_lock();
		list_for_each_entry_rcu(object, &object_list, object_list)
			delete_object_full(object->pointer);
		rcu_read_unlock();
	}
	mutex_unlock(&scan_mutex);
}

static DECLARE_WORK(cleanup_work, kmemleak_do_cleanup);

static void kmemleak_disable(void)
{
	
	if (atomic_cmpxchg(&kmemleak_error, 0, 1))
		return;

	
	atomic_set(&kmemleak_enabled, 0);

	
	if (atomic_read(&kmemleak_initialized))
		schedule_work(&cleanup_work);

	pr_info("Kernel memory leak detector disabled\n");
}

static int kmemleak_boot_config(char *str)
{
	if (!str)
		return -EINVAL;
	if (strcmp(str, "off") == 0)
		kmemleak_disable();
	else if (strcmp(str, "on") == 0)
		kmemleak_skip_disable = 1;
	else
		return -EINVAL;
	return 0;
}
early_param("kmemleak", kmemleak_boot_config);

static void __init print_log_trace(struct early_log *log)
{
	struct stack_trace trace;

	trace.nr_entries = log->trace_len;
	trace.entries = log->trace;

	pr_notice("Early log backtrace:\n");
	print_stack_trace(&trace, 2);
}

#include <mach/board_htc.h>

void __init kmemleak_init(void)
{
	int i;
	unsigned long flags;

	
	if (get_kernel_flag() & KERNEL_FLAG_KMEMLEAK)
		kmemleak_skip_disable = 1;

	if (!kmemleak_skip_disable) {
		atomic_set(&kmemleak_early_log, 0);
		kmemleak_disable();
		return;
	}

	jiffies_min_age = msecs_to_jiffies(MSECS_MIN_AGE);
	jiffies_scan_wait = msecs_to_jiffies(SECS_SCAN_WAIT * 1000);

	object_cache = KMEM_CACHE(kmemleak_object, SLAB_NOLEAKTRACE);
	scan_area_cache = KMEM_CACHE(kmemleak_scan_area, SLAB_NOLEAKTRACE);
	INIT_PRIO_TREE_ROOT(&object_tree_root);

	if (crt_early_log >= ARRAY_SIZE(early_log))
		pr_warning("Early log buffer exceeded (%d), please increase "
			   "DEBUG_KMEMLEAK_EARLY_LOG_SIZE\n", crt_early_log);

	
	local_irq_save(flags);
	atomic_set(&kmemleak_early_log, 0);
	if (atomic_read(&kmemleak_error)) {
		local_irq_restore(flags);
		return;
	} else
		atomic_set(&kmemleak_enabled, 1);
	local_irq_restore(flags);

	for (i = 0; i < crt_early_log; i++) {
		struct early_log *log = &early_log[i];

		switch (log->op_type) {
		case KMEMLEAK_ALLOC:
			early_alloc(log);
			break;
		case KMEMLEAK_ALLOC_PERCPU:
			early_alloc_percpu(log);
			break;
		case KMEMLEAK_FREE:
			kmemleak_free(log->ptr);
			break;
		case KMEMLEAK_FREE_PART:
			kmemleak_free_part(log->ptr, log->size);
			break;
		case KMEMLEAK_FREE_PERCPU:
			kmemleak_free_percpu(log->ptr);
			break;
		case KMEMLEAK_NOT_LEAK:
			kmemleak_not_leak(log->ptr);
			break;
		case KMEMLEAK_IGNORE:
			kmemleak_ignore(log->ptr);
			break;
		case KMEMLEAK_SCAN_AREA:
			kmemleak_scan_area(log->ptr, log->size, GFP_KERNEL);
			break;
		case KMEMLEAK_NO_SCAN:
			kmemleak_no_scan(log->ptr);
			break;
		default:
			kmemleak_warn("Unknown early log operation: %d\n",
				      log->op_type);
		}

		if (atomic_read(&kmemleak_warning)) {
			print_log_trace(log);
			atomic_set(&kmemleak_warning, 0);
		}
	}
}

#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#endif

static int __init kmemleak_late_init(void)
{
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry *p;
#else
	struct dentry *dentry;
#endif

	if (atomic_read(&kmemleak_error)) {
		schedule_work(&cleanup_work);
		return -ENOMEM;
	}

#ifdef CONFIG_PROC_FS
   p = proc_create("kmemleak", S_IRUGO, NULL, &kmemleak_fops);
   if (!p)
       printk("Failed to create the proc kmemleak file\n");
   else
	   printk("create proc kmemleak OK\n");
#else
	dentry = debugfs_create_file("kmemleak", S_IRUGO, NULL, NULL,
				     &kmemleak_fops);
	if (!dentry)
		pr_warning("Failed to create the debugfs kmemleak file\n");
#endif
	mutex_lock(&scan_mutex);
	start_scan_thread();
	mutex_unlock(&scan_mutex);

	pr_info("Kernel memory leak detector initialized\n");
	atomic_set(&kmemleak_initialized, 1);

	return 0;
}
late_initcall(kmemleak_late_init);
