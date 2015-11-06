/*
 * Copyright (C) 2007-2008 Advanced Micro Devices, Inc.
 * Author: Joerg Roedel <joerg.roedel@amd.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#ifndef __LINUX_IOMMU_H
#define __LINUX_IOMMU_H

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/scatterlist.h>

#define IOMMU_READ	(1)
#define IOMMU_WRITE	(2)
#define IOMMU_CACHE	(4) 

struct iommu_ops;
struct bus_type;
struct device;
struct iommu_domain;

#define IOMMU_FAULT_READ	0x0
#define IOMMU_FAULT_WRITE	0x1

typedef int (*iommu_fault_handler_t)(struct iommu_domain *,
				struct device *, unsigned long, int);

struct iommu_domain {
	struct iommu_ops *ops;
	void *priv;
	iommu_fault_handler_t handler;
};

#define IOMMU_CAP_CACHE_COHERENCY	0x1
#define IOMMU_CAP_INTR_REMAP		0x2	

#ifdef CONFIG_IOMMU_API

struct iommu_ops {
	int (*domain_init)(struct iommu_domain *domain, int flags);
	void (*domain_destroy)(struct iommu_domain *domain);
	int (*attach_dev)(struct iommu_domain *domain, struct device *dev);
	void (*detach_dev)(struct iommu_domain *domain, struct device *dev);
	int (*map)(struct iommu_domain *domain, unsigned long iova,
		   phys_addr_t paddr, size_t size, int prot);
	size_t (*unmap)(struct iommu_domain *domain, unsigned long iova,
		     size_t size);
	int (*map_range)(struct iommu_domain *domain, unsigned int iova,
		    struct scatterlist *sg, unsigned int len, int prot);
	int (*unmap_range)(struct iommu_domain *domain, unsigned int iova,
		      unsigned int len);
	phys_addr_t (*iova_to_phys)(struct iommu_domain *domain,
				    unsigned long iova);
	int (*domain_has_cap)(struct iommu_domain *domain,
			      unsigned long cap);
	phys_addr_t (*get_pt_base_addr)(struct iommu_domain *domain);
	int (*device_group)(struct device *dev, unsigned int *groupid);
	unsigned long pgsize_bitmap;
};

extern int bus_set_iommu(struct bus_type *bus, struct iommu_ops *ops);
extern bool iommu_present(struct bus_type *bus);
extern struct iommu_domain *iommu_domain_alloc(struct bus_type *bus, int flags);
extern void iommu_domain_free(struct iommu_domain *domain);
extern int iommu_attach_device(struct iommu_domain *domain,
			       struct device *dev);
extern void iommu_detach_device(struct iommu_domain *domain,
				struct device *dev);
extern int iommu_map(struct iommu_domain *domain, unsigned long iova,
		     phys_addr_t paddr, size_t size, int prot);
extern size_t iommu_unmap(struct iommu_domain *domain, unsigned long iova,
		       size_t size);
extern int iommu_map_range(struct iommu_domain *domain, unsigned int iova,
		    struct scatterlist *sg, unsigned int len, int prot);
extern int iommu_unmap_range(struct iommu_domain *domain, unsigned int iova,
		      unsigned int len);
extern phys_addr_t iommu_iova_to_phys(struct iommu_domain *domain,
				      unsigned long iova);
extern int iommu_domain_has_cap(struct iommu_domain *domain,
				unsigned long cap);
extern phys_addr_t iommu_get_pt_base_addr(struct iommu_domain *domain);
extern void iommu_set_fault_handler(struct iommu_domain *domain,
					iommu_fault_handler_t handler);
extern int iommu_device_group(struct device *dev, unsigned int *groupid);

static inline int report_iommu_fault(struct iommu_domain *domain,
		struct device *dev, unsigned long iova, int flags)
{
	int ret = -ENOSYS;

	if (domain->handler)
		ret = domain->handler(domain, dev, iova, flags);

	return ret;
}

#else 

struct iommu_ops {};

static inline bool iommu_present(struct bus_type *bus)
{
	return false;
}

static inline struct iommu_domain *iommu_domain_alloc(struct bus_type *bus, int flags)
{
	return NULL;
}

static inline void iommu_domain_free(struct iommu_domain *domain)
{
}

static inline int iommu_attach_device(struct iommu_domain *domain,
				      struct device *dev)
{
	return -ENODEV;
}

static inline void iommu_detach_device(struct iommu_domain *domain,
				       struct device *dev)
{
}

static inline int iommu_map(struct iommu_domain *domain, unsigned long iova,
			    phys_addr_t paddr, int gfp_order, int prot)
{
	return -ENODEV;
}

static inline int iommu_unmap(struct iommu_domain *domain, unsigned long iova,
			      int gfp_order)
{
	return -ENODEV;
}

static inline int iommu_map_range(struct iommu_domain *domain,
				  unsigned int iova, struct scatterlist *sg,
				  unsigned int len, int prot)
{
	return -ENODEV;
}

static inline int iommu_unmap_range(struct iommu_domain *domain,
				    unsigned int iova, unsigned int len)
{
	return -ENODEV;
}

static inline phys_addr_t iommu_iova_to_phys(struct iommu_domain *domain,
					     unsigned long iova)
{
	return 0;
}

static inline int domain_has_cap(struct iommu_domain *domain,
				 unsigned long cap)
{
	return 0;
}

static inline phys_addr_t iommu_get_pt_base_addr(struct iommu_domain *domain)
{
	return 0;
}

static inline void iommu_set_fault_handler(struct iommu_domain *domain,
					iommu_fault_handler_t handler)
{
}

static inline int iommu_device_group(struct device *dev, unsigned int *groupid)
{
	return -ENODEV;
}

#endif 

#endif 
