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
#ifndef __MSM_PERIPHERAL_LOADER_H
#define __MSM_PERIPHERAL_LOADER_H

struct device;
struct module;

struct pil_desc {
	const char *name;
	const char *depends_on;
	struct device *dev;
	const struct pil_reset_ops *ops;
	struct module *owner;
	unsigned long proxy_timeout;
};

struct pil_reset_ops {
	int (*init_image)(struct pil_desc *pil, const u8 *metadata,
			  size_t size);
	int (*verify_blob)(struct pil_desc *pil, u32 phy_addr, size_t size);
	int (*proxy_vote)(struct pil_desc *pil);
	int (*auth_and_reset)(struct pil_desc *pil);
	void (*proxy_unvote)(struct pil_desc *pil);
	int (*shutdown)(struct pil_desc *pil);
};

struct pil_device;

extern struct pil_device *msm_pil_register(struct pil_desc *desc);
extern void msm_pil_unregister(struct pil_device *pil);

#endif
