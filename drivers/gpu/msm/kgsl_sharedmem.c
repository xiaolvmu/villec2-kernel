/* Copyright (c) 2002,2007-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/export.h>
#include <linux/vmalloc.h>
#include <linux/memory_alloc.h>
#include <asm/cacheflush.h>
#include <linux/slab.h>
#include <linux/kmemleak.h>
#include <linux/highmem.h>

#include "kgsl.h"
#include "kgsl_sharedmem.h"
#include "kgsl_cffdump.h"
#include "kgsl_device.h"

struct ion_client* kgsl_client = NULL;

struct kgsl_mem_entry_attribute {
	struct attribute attr;
	int memtype;
	ssize_t (*show)(struct kgsl_process_private *priv,
		int type, char *buf);
};

#define to_mem_entry_attr(a) \
container_of(a, struct kgsl_mem_entry_attribute, attr)

#define __MEM_ENTRY_ATTR(_type, _name, _show) \
{ \
	.attr = { .name = __stringify(_name), .mode = 0444 }, \
	.memtype = _type, \
	.show = _show, \
}

#ifdef CONFIG_MSM_KGSL_GPU_USAGE
static ssize_t
gpubusy_show(struct kgsl_process_private *priv, int type, char *buf)
{
	char* tmp = buf;
	int i;

	tmp = (char*)((int)tmp + snprintf(tmp, PAGE_SIZE, "%lld %lld", priv->gputime.total, priv->gputime.busy));
	for(i=0;i<KGSL_MAX_PWRLEVELS;i++)
		tmp = (char*)( (int)tmp + snprintf(tmp, PAGE_SIZE - (int)(tmp-buf), " %lld %lld", priv->gputime_in_state[i].total, priv->gputime_in_state[i].busy));
	tmp = (char*)((int)tmp + snprintf(tmp, PAGE_SIZE, "\n"));
	return (ssize_t)(tmp - buf);
}

static struct kgsl_mem_entry_attribute gpubusy = __MEM_ENTRY_ATTR(0, gpubusy, gpubusy_show);
#endif


struct mem_entry_stats {
	int memtype;
	struct kgsl_mem_entry_attribute attr;
	struct kgsl_mem_entry_attribute max_attr;
};


#define MEM_ENTRY_STAT(_type, _name) \
{ \
	.memtype = _type, \
	.attr = __MEM_ENTRY_ATTR(_type, _name, mem_entry_show), \
	.max_attr = __MEM_ENTRY_ATTR(_type, _name##_max, \
		mem_entry_max_show), \
}



static struct page *kgsl_guard_page;


static struct kgsl_process_private *
_get_priv_from_kobj(struct kobject *kobj)
{
	struct kgsl_process_private *private;
	unsigned long name;

	if (!kobj)
		return NULL;

	if (sscanf(kobj->name, "%ld", &name) != 1)
		return NULL;

	list_for_each_entry(private, &kgsl_driver.process_list, list) {
		if (private->pid == name)
			return private;
	}

	return NULL;
}


static ssize_t
mem_entry_show(struct kgsl_process_private *priv, int type, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", priv->stats[type].cur);
}


static ssize_t
mem_entry_max_show(struct kgsl_process_private *priv, int type, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", priv->stats[type].max);
}


static void mem_entry_sysfs_release(struct kobject *kobj)
{
}

static ssize_t mem_entry_sysfs_show(struct kobject *kobj,
	struct attribute *attr, char *buf)
{
	struct kgsl_mem_entry_attribute *pattr = to_mem_entry_attr(attr);
	struct kgsl_process_private *priv;
	ssize_t ret;

	mutex_lock(&kgsl_driver.process_mutex);
	priv = _get_priv_from_kobj(kobj);

	if (priv && pattr->show)
		ret = pattr->show(priv, pattr->memtype, buf);
	else
		ret = -EIO;

	mutex_unlock(&kgsl_driver.process_mutex);
	return ret;
}

static const struct sysfs_ops mem_entry_sysfs_ops = {
	.show = mem_entry_sysfs_show,
};

static struct kobj_type ktype_mem_entry = {
	.sysfs_ops = &mem_entry_sysfs_ops,
	.default_attrs = NULL,
	.release = mem_entry_sysfs_release
};

static struct mem_entry_stats mem_stats[] = {
	MEM_ENTRY_STAT(KGSL_MEM_ENTRY_KERNEL, kernel),
#ifdef CONFIG_ANDROID_PMEM
	MEM_ENTRY_STAT(KGSL_MEM_ENTRY_PMEM, pmem),
#endif
#ifdef CONFIG_ASHMEM
	MEM_ENTRY_STAT(KGSL_MEM_ENTRY_ASHMEM, ashmem),
#endif
	MEM_ENTRY_STAT(KGSL_MEM_ENTRY_USER, user),
#ifdef CONFIG_ION
	MEM_ENTRY_STAT(KGSL_MEM_ENTRY_ION, ion),
#endif
};

void
kgsl_process_uninit_sysfs(struct kgsl_process_private *private)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mem_stats); i++) {
		sysfs_remove_file(&private->kobj, &mem_stats[i].attr.attr);
		sysfs_remove_file(&private->kobj,
			&mem_stats[i].max_attr.attr);
	}

#ifdef CONFIG_MSM_KGSL_GPU_USAGE
	sysfs_remove_file(&private->kobj, &gpubusy.attr);
#endif
	kobject_put(&private->kobj);
}

void
kgsl_process_init_sysfs(struct kgsl_process_private *private)
{
	unsigned char name[16];
	int i, ret;

	snprintf(name, sizeof(name), "%d", private->pid);

	if (kobject_init_and_add(&private->kobj, &ktype_mem_entry,
		kgsl_driver.prockobj, name))
		return;

	for (i = 0; i < ARRAY_SIZE(mem_stats); i++) {

		ret = sysfs_create_file(&private->kobj,
			&mem_stats[i].attr.attr);
		ret = sysfs_create_file(&private->kobj,
			&mem_stats[i].max_attr.attr);
	}
#ifdef CONFIG_MSM_KGSL_GPU_USAGE
	ret = sysfs_create_file(&private->kobj, &gpubusy.attr);
#endif
}

static int kgsl_drv_memstat_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	unsigned int val = 0;

	if (!strncmp(attr->attr.name, "vmalloc", 7))
		val = kgsl_driver.stats.vmalloc;
	else if (!strncmp(attr->attr.name, "vmalloc_max", 11))
		val = kgsl_driver.stats.vmalloc_max;
	else if (!strncmp(attr->attr.name, "page_alloc", 10))
		val = kgsl_driver.stats.page_alloc;
	else if (!strncmp(attr->attr.name, "page_alloc_max", 14))
		val = kgsl_driver.stats.page_alloc_max;
	else if (!strncmp(attr->attr.name, "coherent", 8))
		val = kgsl_driver.stats.coherent;
	else if (!strncmp(attr->attr.name, "coherent_max", 12))
		val = kgsl_driver.stats.coherent_max;
	else if (!strncmp(attr->attr.name, "mapped", 6))
		val = kgsl_driver.stats.mapped;
	else if (!strncmp(attr->attr.name, "mapped_max", 10))
		val = kgsl_driver.stats.mapped_max;

	return snprintf(buf, PAGE_SIZE, "%u\n", val);
}

static int kgsl_drv_histogram_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	int len = 0;
	int i;

	for (i = 0; i < 16; i++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%d ",
			kgsl_driver.stats.histogram[i]);

	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	return len;
}

DEVICE_ATTR(vmalloc, 0444, kgsl_drv_memstat_show, NULL);
DEVICE_ATTR(vmalloc_max, 0444, kgsl_drv_memstat_show, NULL);
DEVICE_ATTR(page_alloc, 0444, kgsl_drv_memstat_show, NULL);
DEVICE_ATTR(page_alloc_max, 0444, kgsl_drv_memstat_show, NULL);
DEVICE_ATTR(coherent, 0444, kgsl_drv_memstat_show, NULL);
DEVICE_ATTR(coherent_max, 0444, kgsl_drv_memstat_show, NULL);
DEVICE_ATTR(mapped, 0444, kgsl_drv_memstat_show, NULL);
DEVICE_ATTR(mapped_max, 0444, kgsl_drv_memstat_show, NULL);
DEVICE_ATTR(histogram, 0444, kgsl_drv_histogram_show, NULL);

static const struct device_attribute *drv_attr_list[] = {
	&dev_attr_vmalloc,
	&dev_attr_vmalloc_max,
	&dev_attr_page_alloc,
	&dev_attr_page_alloc_max,
	&dev_attr_coherent,
	&dev_attr_coherent_max,
	&dev_attr_mapped,
	&dev_attr_mapped_max,
	&dev_attr_histogram,
	NULL
};

void
kgsl_sharedmem_uninit_sysfs(void)
{
	kgsl_remove_device_sysfs_files(&kgsl_driver.virtdev, drv_attr_list);
}

int
kgsl_sharedmem_init_sysfs(void)
{
	return kgsl_create_device_sysfs_files(&kgsl_driver.virtdev,
		drv_attr_list);
}

#ifdef CONFIG_OUTER_CACHE
static void _outer_cache_range_op(int op, unsigned long addr, size_t size)
{
	switch (op) {
	case KGSL_CACHE_OP_FLUSH:
		outer_flush_range(addr, addr + size);
		break;
	case KGSL_CACHE_OP_CLEAN:
		outer_clean_range(addr, addr + size);
		break;
	case KGSL_CACHE_OP_INV:
		outer_inv_range(addr, addr + size);
		break;
	}
}

static void outer_cache_range_op_sg(struct scatterlist *sg, int sglen, int op)
{
	struct scatterlist *s;
	int i;

	for_each_sg(sg, s, sglen, i) {
		unsigned int paddr = kgsl_get_sg_pa(s);
		_outer_cache_range_op(op, paddr, s->length);
	}
}

#else
static void outer_cache_range_op_sg(struct scatterlist *sg, int sglen, int op)
{
}
#endif

static int kgsl_ion_alloc_vmfault(struct kgsl_memdesc *memdesc,
				struct vm_area_struct *vma,
				struct vm_fault *vmf)
{
	unsigned long offset, pfn;
	int ret;

	offset = ((unsigned long) vmf->virtual_address - vma->vm_start) >> PAGE_SHIFT;

	pfn = (memdesc->sg[0].dma_address >> PAGE_SHIFT) + offset;
	ret = vm_insert_pfn(vma, (unsigned long) vmf->virtual_address, pfn);

	if (ret == -ENOMEM || ret == -EAGAIN)
		return VM_FAULT_OOM;
	else if (ret == -EFAULT)
		return VM_FAULT_SIGBUS;

	return 0;
}

static int kgsl_ion_alloc_vmflags(struct kgsl_memdesc *memdesc)
{
	return VM_RESERVED | VM_DONTEXPAND;
}

static void kgsl_ion_alloc_free(struct kgsl_memdesc *memdesc)
{
	kgsl_driver.stats.pre_alloc -= memdesc->size;
	if (memdesc->handle)
		ion_free(kgsl_client, memdesc->handle);

	if (memdesc->hostptr) {
		iounmap(memdesc->hostptr);
		kgsl_driver.stats.vmalloc -= memdesc->size;
	}

	if (memdesc->private)
		kgsl_process_sub_stats(memdesc->private, KGSL_MEM_ENTRY_PRE_ALLOC, memdesc->size);
	else
		kgsl_driver.stats.pre_alloc_kernel -= memdesc->size;
}

static int kgsl_ion_alloc_map_kernel(struct kgsl_memdesc *memdesc)
{
	if (!memdesc->hostptr) {
		memdesc->hostptr = ioremap(memdesc->sg[0].dma_address, memdesc->sg[0].length);
		if(IS_ERR_OR_NULL(memdesc->hostptr)) {
			KGSL_CORE_ERR("kgsl: ion ioremap failed\n");
			return -ENOMEM;
		}
		KGSL_STATS_ADD(memdesc->size, kgsl_driver.stats.vmalloc,
                kgsl_driver.stats.vmalloc_max);
	}

	return 0;
}

static int kgsl_page_alloc_vmfault(struct kgsl_memdesc *memdesc,
				struct vm_area_struct *vma,
				struct vm_fault *vmf)
{
	unsigned long offset;
	struct page *page;
	int i;

	offset = (unsigned long) vmf->virtual_address - vma->vm_start;

	i = offset >> PAGE_SHIFT;
	page = sg_page(&memdesc->sg[i]);
	if (page == NULL)
		return VM_FAULT_SIGBUS;

	get_page(page);

	vmf->page = page;
	return 0;
}

static int kgsl_page_alloc_vmflags(struct kgsl_memdesc *memdesc)
{
	return VM_RESERVED | VM_DONTEXPAND;
}

static void kgsl_page_alloc_free(struct kgsl_memdesc *memdesc)
{
	int i = 0;
	struct scatterlist *sg;
	int sglen = memdesc->sglen;

	
	if (memdesc->flags & KGSL_MEMDESC_GUARD_PAGE)
		sglen--;

	kgsl_driver.stats.page_alloc -= memdesc->size;

	if (memdesc->hostptr) {
		vunmap(memdesc->hostptr);
		kgsl_driver.stats.vmalloc -= memdesc->size;
	}
	if (memdesc->sg)
		for_each_sg(memdesc->sg, sg, sglen, i)
			__free_page(sg_page(sg));

	if (memdesc->private)
		kgsl_process_sub_stats(memdesc->private, KGSL_MEM_ENTRY_PAGE_ALLOC, memdesc->size);
	else
		kgsl_driver.stats.page_alloc_kernel -= memdesc->size;
}

static int kgsl_contiguous_vmflags(struct kgsl_memdesc *memdesc)
{
	return VM_RESERVED | VM_IO | VM_PFNMAP | VM_DONTEXPAND;
}

static int kgsl_page_alloc_map_kernel(struct kgsl_memdesc *memdesc)
{
	if (!memdesc->hostptr) {
		pgprot_t page_prot = pgprot_writecombine(PAGE_KERNEL);
		struct page **pages = NULL;
		struct scatterlist *sg;
		int sglen = memdesc->sglen;
		int i;

		
		if (memdesc->flags & KGSL_MEMDESC_GUARD_PAGE)
			sglen--;

		
		pages = vmalloc(sglen * sizeof(struct page *));
		if (!pages) {
			KGSL_CORE_ERR("vmalloc(%d) failed\n",
				sglen * sizeof(struct page *));
			return -ENOMEM;
		}
		for_each_sg(memdesc->sg, sg, sglen, i)
			pages[i] = sg_page(sg);
		memdesc->hostptr = vmap(pages, sglen,
					VM_IOREMAP, page_prot);
		KGSL_STATS_ADD(memdesc->size, kgsl_driver.stats.vmalloc,
				kgsl_driver.stats.vmalloc_max);
		vfree(pages);
	}
	if (!memdesc->hostptr)
		return -ENOMEM;

	return 0;
}

static int kgsl_contiguous_vmfault(struct kgsl_memdesc *memdesc,
				struct vm_area_struct *vma,
				struct vm_fault *vmf)
{
	unsigned long offset, pfn;
	int ret;

	offset = ((unsigned long) vmf->virtual_address - vma->vm_start) >>
		PAGE_SHIFT;

	pfn = (memdesc->physaddr >> PAGE_SHIFT) + offset;
	ret = vm_insert_pfn(vma, (unsigned long) vmf->virtual_address, pfn);

	if (ret == -ENOMEM || ret == -EAGAIN)
		return VM_FAULT_OOM;
	else if (ret == -EFAULT)
		return VM_FAULT_SIGBUS;

	return VM_FAULT_NOPAGE;
}

static void kgsl_ebimem_free(struct kgsl_memdesc *memdesc)

{
	kgsl_driver.stats.coherent -= memdesc->size;
	if (memdesc->hostptr)
		iounmap(memdesc->hostptr);

	free_contiguous_memory_by_paddr(memdesc->physaddr);
}

static void kgsl_coherent_free(struct kgsl_memdesc *memdesc)
{
	kgsl_driver.stats.coherent -= memdesc->size;
	dma_free_coherent(NULL, memdesc->size,
			  memdesc->hostptr, memdesc->physaddr);
}

struct kgsl_memdesc_ops kgsl_page_alloc_ops = {
	.free = kgsl_page_alloc_free,
	.vmflags = kgsl_page_alloc_vmflags,
	.vmfault = kgsl_page_alloc_vmfault,
	.map_kernel_mem = kgsl_page_alloc_map_kernel,
};
EXPORT_SYMBOL(kgsl_page_alloc_ops);

struct kgsl_memdesc_ops kgsl_ion_alloc_ops = {
	.free = kgsl_ion_alloc_free,
	.vmflags = kgsl_ion_alloc_vmflags,
	.vmfault = kgsl_ion_alloc_vmfault,
	.map_kernel_mem = kgsl_ion_alloc_map_kernel,
};
EXPORT_SYMBOL(kgsl_ion_alloc_ops);


static struct kgsl_memdesc_ops kgsl_ebimem_ops = {
	.free = kgsl_ebimem_free,
	.vmflags = kgsl_contiguous_vmflags,
	.vmfault = kgsl_contiguous_vmfault,
};

static struct kgsl_memdesc_ops kgsl_coherent_ops = {
	.free = kgsl_coherent_free,
};

void kgsl_cache_range_op(struct kgsl_memdesc *memdesc, int op)
{
	void *addr = memdesc->hostptr;
	int size = memdesc->size;

	switch (op) {
	case KGSL_CACHE_OP_FLUSH:
		dmac_flush_range(addr, addr + size);
		break;
	case KGSL_CACHE_OP_CLEAN:
		dmac_clean_range(addr, addr + size);
		break;
	case KGSL_CACHE_OP_INV:
		dmac_inv_range(addr, addr + size);
		break;
	}

	outer_cache_range_op_sg(memdesc->sg, memdesc->sglen, op);
}
EXPORT_SYMBOL(kgsl_cache_range_op);

static int
_kgsl_sharedmem_page_alloc(struct kgsl_memdesc *memdesc,
			struct kgsl_pagetable *pagetable,
			size_t size, unsigned int protflags)
{
	int i, order, ret = 0;
	int sglen = PAGE_ALIGN(size) / PAGE_SIZE;
	struct page **pages = NULL;
	pgprot_t page_prot = pgprot_writecombine(PAGE_KERNEL);
	void *ptr;


	if (kgsl_mmu_get_mmutype() == KGSL_MMU_TYPE_IOMMU)
		sglen++;

	memdesc->size = size;
	memdesc->pagetable = pagetable;
	memdesc->priv = KGSL_MEMFLAGS_CACHED;
	memdesc->ops = &kgsl_page_alloc_ops;

	memdesc->sg = kgsl_sg_alloc(sglen);

	if (memdesc->sg == NULL) {
		KGSL_CORE_ERR("vmalloc(%d) failed\n",
			sglen * sizeof(struct scatterlist));
		ret = -ENOMEM;
		goto done;
	}


	pages = kmalloc(sglen * sizeof(struct page *), GFP_KERNEL);

	if (pages == NULL) {
		KGSL_CORE_ERR("kmalloc (%d) failed\n",
			sglen * sizeof(struct page *));
		ret = -ENOMEM;
		goto done;
	}

	kmemleak_not_leak(memdesc->sg);

	memdesc->sglen = sglen;
	sg_init_table(memdesc->sg, sglen);

	for (i = 0; i < PAGE_ALIGN(size) / PAGE_SIZE; i++) {


		pages[i] = alloc_page(GFP_KERNEL | __GFP_HIGHMEM);
		if (pages[i] == NULL) {
			ret = -ENOMEM;
			memdesc->sglen = i;
			goto done;
		}

		sg_set_page(&memdesc->sg[i], pages[i], PAGE_SIZE, 0);
	}

	

	if (kgsl_mmu_get_mmutype() == KGSL_MMU_TYPE_IOMMU) {

		if (kgsl_guard_page == NULL)
			kgsl_guard_page = alloc_page(GFP_KERNEL | __GFP_ZERO |
				__GFP_HIGHMEM);

		if (kgsl_guard_page != NULL) {
			sg_set_page(&memdesc->sg[sglen - 1], kgsl_guard_page,
				PAGE_SIZE, 0);
			memdesc->flags |= KGSL_MEMDESC_GUARD_PAGE;
		} else
			memdesc->sglen--;
	}


	ptr = vmap(pages, i, VM_IOREMAP, page_prot);

	if (ptr != NULL) {
		memset(ptr, 0, memdesc->size);
		dmac_flush_range(ptr, ptr + memdesc->size);
		vunmap(ptr);
	} else {
		int j;

		

		for (j = 0; j < i; j++) {
			ptr = kmap_atomic(pages[j]);
			memset(ptr, 0, PAGE_SIZE);
			dmac_flush_range(ptr, ptr + PAGE_SIZE);
			kunmap_atomic(ptr);
		}
	}

	outer_cache_range_op_sg(memdesc->sg, memdesc->sglen,
				KGSL_CACHE_OP_FLUSH);

	ret = kgsl_mmu_map(pagetable, memdesc, protflags);

	if (ret)
		goto done;

	order = get_order(size);

	if (order < 16)
		kgsl_driver.stats.histogram[order]++;

done:
	kfree(pages);

	KGSL_STATS_ADD(size, kgsl_driver.stats.page_alloc,
		kgsl_driver.stats.page_alloc_max);

	if (ret)
		kgsl_sharedmem_free(memdesc);

	return ret;
}

int
kgsl_sharedmem_page_alloc(struct kgsl_memdesc *memdesc,
		       struct kgsl_pagetable *pagetable, size_t size)
{
	int ret = 0;
	BUG_ON(size == 0);

	size = ALIGN(size, PAGE_SIZE * 2);

	kgsl_driver.stats.page_alloc_kernel += size;
	ret =  _kgsl_sharedmem_page_alloc(memdesc, pagetable, size,
		GSL_PT_PAGE_RV | GSL_PT_PAGE_WV);
	if (!ret)
		ret = kgsl_page_alloc_map_kernel(memdesc);
	if (ret) {
		
		kgsl_driver.stats.page_alloc_kernel += size;
		kgsl_sharedmem_free(memdesc);
	}
	return ret;
}
EXPORT_SYMBOL(kgsl_sharedmem_page_alloc);

int
kgsl_sharedmem_page_alloc_user(struct kgsl_memdesc *memdesc,
				struct kgsl_process_private *private,
			    struct kgsl_pagetable *pagetable,
			    size_t size, int flags)
{
	unsigned int protflags;
	int ret = 0;

	if (size == 0)
		return -EINVAL;

	protflags = GSL_PT_PAGE_RV;
	if (!(flags & KGSL_MEMFLAGS_GPUREADONLY))
		protflags |= GSL_PT_PAGE_WV;

	ret = _kgsl_sharedmem_page_alloc(memdesc, pagetable, size,
			protflags);

	if (ret == 0 && private)
		kgsl_process_add_stats(private, KGSL_MEM_ENTRY_PAGE_ALLOC, size);

	return ret;
}
EXPORT_SYMBOL(kgsl_sharedmem_page_alloc_user);

static int
_kgsl_sharedmem_ion_alloc(struct kgsl_memdesc *memdesc,
				struct kgsl_pagetable *pagetable,
				size_t size, unsigned int protflags)
{
	int order, ret = 0;
	int sglen = 1;
	void *ptr;
	struct ion_handle *handle = NULL;
	ion_phys_addr_t pa = 0;
	size_t len = 0;


	
	

	memdesc->size = size;
	memdesc->pagetable = pagetable;
	memdesc->priv = KGSL_MEMFLAGS_CACHED;
	memdesc->ops = &kgsl_ion_alloc_ops;

	memdesc->sg = kgsl_sg_alloc(sglen);

	if (memdesc->sg == NULL) {
		KGSL_CORE_ERR("kgsl_sg_alloc vmalloc(%d) failed\n",
		sglen * sizeof(struct scatterlist));
		ret = -ENOMEM;
		goto done;
	}

	kmemleak_not_leak(memdesc->sg);

	memdesc->sglen = sglen;
	sg_init_table(memdesc->sg, sglen);

	if (kgsl_client == NULL)
		kgsl_client = msm_ion_client_create(-1, "KGSL");

	handle = ion_alloc(kgsl_client, size, SZ_4K, 0x1 << ION_SF_HEAP_ID);
	if (IS_ERR_OR_NULL(handle)) {
		ret = -ENOMEM;
		goto done;
	}

	if (ion_phys(kgsl_client, handle, &pa, &len)) {
		KGSL_CORE_ERR("kgsl: ion_phys() failed\n");
		ret = -ENOMEM;
		goto done;
	}

	memdesc->handle = handle;

	memdesc->sg[0].length = memdesc->size;
	memdesc->sg[0].offset = 0;
	memdesc->sg[0].dma_address = pa;

	
	


	ptr = ioremap(pa, memdesc->size);

	if (ptr != NULL) {
		memset(ptr, 0, memdesc->size);
		dmac_flush_range(ptr, ptr + memdesc->size);
		iounmap(ptr);
	}

	outer_cache_range_op_sg(memdesc->sg, memdesc->sglen, KGSL_CACHE_OP_FLUSH);

	ret = kgsl_mmu_map(pagetable, memdesc, protflags);

	if (ret) {
		KGSL_CORE_ERR("kgsl: kgsl_mmu_map failed\n");
		ret = -ENOMEM;
		goto done;
	}

	order = get_order(size);

	if (order < 16)
		kgsl_driver.stats.histogram[order]++;

done:
	KGSL_STATS_ADD(size, kgsl_driver.stats.pre_alloc, kgsl_driver.stats.pre_alloc_max);

	if (ret)
		kgsl_sharedmem_free(memdesc);

	return ret;
}

int
kgsl_sharedmem_ion_alloc(struct kgsl_memdesc *memdesc,
                struct kgsl_pagetable *pagetable,
                size_t size)
{
	int ret;

	BUG_ON(size == 0);
	size = PAGE_ALIGN(size);

	kgsl_driver.stats.pre_alloc_kernel += size;
	ret = _kgsl_sharedmem_ion_alloc(memdesc, pagetable, size,
		GSL_PT_PAGE_RV | GSL_PT_PAGE_WV);

	if (!ret)
		ret = kgsl_ion_alloc_map_kernel(memdesc);

	if (ret) {
		
		kgsl_driver.stats.pre_alloc_kernel += size;
		kgsl_sharedmem_free(memdesc);
	}
	return ret;
}
EXPORT_SYMBOL(kgsl_sharedmem_ion_alloc);

int
kgsl_sharedmem_ion_alloc_user(struct kgsl_memdesc *memdesc,
                struct kgsl_process_private *private,
				struct kgsl_pagetable *pagetable,
				size_t size, int flags)
{
	unsigned int protflags;
	int ret = 0;

	BUG_ON(size == 0);

	size = PAGE_ALIGN(size);

	protflags = GSL_PT_PAGE_RV;
	if (!(flags & KGSL_MEMFLAGS_GPUREADONLY))
		protflags |= GSL_PT_PAGE_WV;

	ret = _kgsl_sharedmem_ion_alloc(memdesc, pagetable, size,
			protflags);

	if (ret == 0 && private)
		kgsl_process_add_stats(private, KGSL_MEM_ENTRY_PRE_ALLOC, size);

	return ret;
}
EXPORT_SYMBOL(kgsl_sharedmem_ion_alloc_user);

int
kgsl_sharedmem_alloc_coherent(struct kgsl_memdesc *memdesc, size_t size)
{
	int result = 0;

	size = ALIGN(size, PAGE_SIZE);

	memdesc->size = size;
	memdesc->ops = &kgsl_coherent_ops;

	memdesc->hostptr = dma_alloc_coherent(NULL, size, &memdesc->physaddr,
					      GFP_KERNEL);
	if (memdesc->hostptr == NULL) {
		KGSL_CORE_ERR("dma_alloc_coherent(%d) failed\n", size);
		result = -ENOMEM;
		goto err;
	}

	result = memdesc_sg_phys(memdesc, memdesc->physaddr, size);
	if (result)
		goto err;

	

	KGSL_STATS_ADD(size, kgsl_driver.stats.coherent,
		       kgsl_driver.stats.coherent_max);

err:
	if (result)
		kgsl_sharedmem_free(memdesc);

	return result;
}
EXPORT_SYMBOL(kgsl_sharedmem_alloc_coherent);

void kgsl_sharedmem_free(struct kgsl_memdesc *memdesc)
{
	if (memdesc == NULL || memdesc->size == 0)
		return;

	if (memdesc->gpuaddr)
		kgsl_mmu_unmap(memdesc->pagetable, memdesc);

	if (memdesc->ops && memdesc->ops->free)
		memdesc->ops->free(memdesc);

	kgsl_sg_free(memdesc->sg, memdesc->sglen);

	memset(memdesc, 0, sizeof(*memdesc));
}
EXPORT_SYMBOL(kgsl_sharedmem_free);

static int
_kgsl_sharedmem_ebimem(struct kgsl_memdesc *memdesc,
			struct kgsl_pagetable *pagetable, size_t size)
{
	int result = 0;

	memdesc->size = size;
	memdesc->pagetable = pagetable;
	memdesc->ops = &kgsl_ebimem_ops;
	memdesc->physaddr = allocate_contiguous_ebi_nomap(size, SZ_8K);

	if (memdesc->physaddr == 0) {
		KGSL_CORE_ERR("allocate_contiguous_ebi_nomap(%d) failed\n",
			size);
		return -ENOMEM;
	}

	result = memdesc_sg_phys(memdesc, memdesc->physaddr, size);

	if (result)
		goto err;

	result = kgsl_mmu_map(pagetable, memdesc,
		GSL_PT_PAGE_RV | GSL_PT_PAGE_WV);

	if (result)
		goto err;

	KGSL_STATS_ADD(size, kgsl_driver.stats.coherent,
		kgsl_driver.stats.coherent_max);

err:
	if (result)
		kgsl_sharedmem_free(memdesc);

	return result;
}

int
kgsl_sharedmem_ebimem_user(struct kgsl_memdesc *memdesc,
			struct kgsl_pagetable *pagetable,
			size_t size, int flags)
{
	size = ALIGN(size, PAGE_SIZE);
	return _kgsl_sharedmem_ebimem(memdesc, pagetable, size);
}
EXPORT_SYMBOL(kgsl_sharedmem_ebimem_user);

int
kgsl_sharedmem_ebimem(struct kgsl_memdesc *memdesc,
		struct kgsl_pagetable *pagetable, size_t size)
{
	int result;
	size = ALIGN(size, 8192);
	result = _kgsl_sharedmem_ebimem(memdesc, pagetable, size);

	if (result)
		return result;

	memdesc->hostptr = ioremap(memdesc->physaddr, size);

	if (memdesc->hostptr == NULL) {
		KGSL_CORE_ERR("ioremap failed\n");
		kgsl_sharedmem_free(memdesc);
		return -ENOMEM;
	}

	return 0;
}
EXPORT_SYMBOL(kgsl_sharedmem_ebimem);

int
kgsl_sharedmem_readl(const struct kgsl_memdesc *memdesc,
			uint32_t *dst,
			unsigned int offsetbytes)
{
	uint32_t *src;
	BUG_ON(memdesc == NULL || memdesc->hostptr == NULL || dst == NULL);
	WARN_ON(offsetbytes % sizeof(uint32_t) != 0);
	if (offsetbytes % sizeof(uint32_t) != 0)
		return -EINVAL;

	WARN_ON(offsetbytes + sizeof(uint32_t) > memdesc->size);
	if (offsetbytes + sizeof(uint32_t) > memdesc->size)
		return -ERANGE;
	src = (uint32_t *)(memdesc->hostptr + offsetbytes);
	*dst = *src;
	return 0;
}
EXPORT_SYMBOL(kgsl_sharedmem_readl);

int
kgsl_sharedmem_writel(const struct kgsl_memdesc *memdesc,
			unsigned int offsetbytes,
			uint32_t src)
{
	uint32_t *dst;
	BUG_ON(memdesc == NULL || memdesc->hostptr == NULL);
	WARN_ON(offsetbytes % sizeof(uint32_t) != 0);
	if (offsetbytes % sizeof(uint32_t) != 0)
		return -EINVAL;

	WARN_ON(offsetbytes + sizeof(uint32_t) > memdesc->size);
	if (offsetbytes + sizeof(uint32_t) > memdesc->size)
		return -ERANGE;
	kgsl_cffdump_setmem(memdesc->gpuaddr + offsetbytes,
		src, sizeof(uint32_t));
	dst = (uint32_t *)(memdesc->hostptr + offsetbytes);
	*dst = src;
	return 0;
}
EXPORT_SYMBOL(kgsl_sharedmem_writel);

int
kgsl_sharedmem_set(const struct kgsl_memdesc *memdesc, unsigned int offsetbytes,
			unsigned int value, unsigned int sizebytes)
{
	BUG_ON(memdesc == NULL || memdesc->hostptr == NULL);
	BUG_ON(offsetbytes + sizebytes > memdesc->size);

	kgsl_cffdump_setmem(memdesc->gpuaddr + offsetbytes, value,
			    sizebytes);
	memset(memdesc->hostptr + offsetbytes, value, sizebytes);
	return 0;
}
EXPORT_SYMBOL(kgsl_sharedmem_set);

int
kgsl_sharedmem_map_vma(struct vm_area_struct *vma,
			const struct kgsl_memdesc *memdesc)
{
	unsigned long addr = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	int ret, i = 0;

	if (!memdesc->sg || (size != memdesc->size) ||
		(memdesc->sglen != (size / PAGE_SIZE)))
		return -EINVAL;

	for (; addr < vma->vm_end; addr += PAGE_SIZE, i++) {
		ret = vm_insert_page(vma, addr, sg_page(&memdesc->sg[i]));
		if (ret)
			return ret;
	}
	return 0;
}
EXPORT_SYMBOL(kgsl_sharedmem_map_vma);
