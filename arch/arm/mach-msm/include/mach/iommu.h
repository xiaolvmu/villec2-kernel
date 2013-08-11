/* Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef MSM_IOMMU_H
#define MSM_IOMMU_H

#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <mach/socinfo.h>

extern pgprot_t     pgprot_kernel;
extern struct platform_device *msm_iommu_root_dev;

#define MSM_IOMMU_DOMAIN_PT_CACHEABLE	0x1

#define MSM_IOMMU_CP_MASK		0x03

#define MAX_NUM_MIDS	32

struct msm_iommu_dev {
	const char *name;
	int ncb;
	int ttbr_split;
};

struct msm_iommu_ctx_dev {
	const char *name;
	int num;
	int mids[MAX_NUM_MIDS];
};


struct msm_iommu_drvdata {
	void __iomem *base;
	int ncb;
	int ttbr_split;
	struct clk *clk;
	struct clk *pclk;
	const char *name;
	struct regulator *gdsc;
};

struct msm_iommu_ctx_drvdata {
	int num;
	struct platform_device *pdev;
	struct list_head attached_elm;
	struct iommu_domain *attached_domain;
	const char *name;
};

irqreturn_t msm_iommu_fault_handler(int irq, void *dev_id);
irqreturn_t msm_iommu_fault_handler_v2(int irq, void *dev_id);

#ifdef CONFIG_MSM_IOMMU
struct device *msm_iommu_get_ctx(const char *ctx_name);
#else
static inline struct device *msm_iommu_get_ctx(const char *ctx_name)
{
	return NULL;
}
#endif

#endif

static inline int msm_soc_version_supports_iommu_v1(void)
{
#ifdef CONFIG_OF
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "qcom,msm-smmu-v2");
	if (node) {
		of_node_put(node);
		return 0;
	}
#endif
	if (cpu_is_msm8960() &&
	    SOCINFO_VERSION_MAJOR(socinfo_get_version()) < 2)
		return 0;

	if (cpu_is_msm8x60() &&
	    (SOCINFO_VERSION_MAJOR(socinfo_get_version()) != 2 ||
	    SOCINFO_VERSION_MINOR(socinfo_get_version()) < 1))	{
		return 0;
	}
	return 1;
}
