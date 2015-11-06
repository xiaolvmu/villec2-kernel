#ifndef _LIBFDT_H
#define _LIBFDT_H
/*
 * libfdt - Flat Device Tree manipulation
 * Copyright (C) 2006 David Gibson, IBM Corporation.
 *
 * libfdt is dual licensed: you can use it either under the terms of
 * the GPL, or the BSD license, at your option.
 *
 *  a) This library is free software; you can redistribute it and/or
 *     modify it under the terms of the GNU General Public License as
 *     published by the Free Software Foundation; either version 2 of the
 *     License, or (at your option) any later version.
 *
 *     This library is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public
 *     License along with this library; if not, write to the Free
 *     Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
 *     MA 02110-1301 USA
 *
 * Alternatively,
 *
 *  b) Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *     1. Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *     2. Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 *     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *     CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *     INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *     MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 *     CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *     SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *     NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *     HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *     CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *     OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 *     EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <libfdt_env.h>
#include <fdt.h>

#define FDT_FIRST_SUPPORTED_VERSION	0x10
#define FDT_LAST_SUPPORTED_VERSION	0x11

#define FDT_ERR_NOTFOUND	1
	
#define FDT_ERR_EXISTS		2
#define FDT_ERR_NOSPACE		3

#define FDT_ERR_BADOFFSET	4
#define FDT_ERR_BADPATH		5
#define FDT_ERR_BADPHANDLE	6
#define FDT_ERR_BADSTATE	7

#define FDT_ERR_TRUNCATED	8
#define FDT_ERR_BADMAGIC	9
#define FDT_ERR_BADVERSION	10
#define FDT_ERR_BADSTRUCTURE	11
#define FDT_ERR_BADLAYOUT	12

#define FDT_ERR_INTERNAL	13

#define FDT_ERR_MAX		13


const void *fdt_offset_ptr(const void *fdt, int offset, int checklen);
static inline void *fdt_offset_ptr_w(void *fdt, int offset, int checklen)
{
	return (void *)(uintptr_t)fdt_offset_ptr(fdt, offset, checklen);
}

uint32_t fdt_next_tag(const void *fdt, int offset, int *nextoffset);


int fdt_next_node(const void *fdt, int offset, int *depth);


#define fdt_get_header(fdt, field) \
	(fdt32_to_cpu(((const struct fdt_header *)(fdt))->field))
#define fdt_magic(fdt) 			(fdt_get_header(fdt, magic))
#define fdt_totalsize(fdt)		(fdt_get_header(fdt, totalsize))
#define fdt_off_dt_struct(fdt)		(fdt_get_header(fdt, off_dt_struct))
#define fdt_off_dt_strings(fdt)		(fdt_get_header(fdt, off_dt_strings))
#define fdt_off_mem_rsvmap(fdt)		(fdt_get_header(fdt, off_mem_rsvmap))
#define fdt_version(fdt)		(fdt_get_header(fdt, version))
#define fdt_last_comp_version(fdt) 	(fdt_get_header(fdt, last_comp_version))
#define fdt_boot_cpuid_phys(fdt) 	(fdt_get_header(fdt, boot_cpuid_phys))
#define fdt_size_dt_strings(fdt) 	(fdt_get_header(fdt, size_dt_strings))
#define fdt_size_dt_struct(fdt)		(fdt_get_header(fdt, size_dt_struct))

#define __fdt_set_hdr(name) \
	static inline void fdt_set_##name(void *fdt, uint32_t val) \
	{ \
		struct fdt_header *fdth = fdt; \
		fdth->name = cpu_to_fdt32(val); \
	}
__fdt_set_hdr(magic);
__fdt_set_hdr(totalsize);
__fdt_set_hdr(off_dt_struct);
__fdt_set_hdr(off_dt_strings);
__fdt_set_hdr(off_mem_rsvmap);
__fdt_set_hdr(version);
__fdt_set_hdr(last_comp_version);
__fdt_set_hdr(boot_cpuid_phys);
__fdt_set_hdr(size_dt_strings);
__fdt_set_hdr(size_dt_struct);
#undef __fdt_set_hdr

int fdt_check_header(const void *fdt);

int fdt_move(const void *fdt, void *buf, int bufsize);


const char *fdt_string(const void *fdt, int stroffset);

int fdt_num_mem_rsv(const void *fdt);

int fdt_get_mem_rsv(const void *fdt, int n, uint64_t *address, uint64_t *size);

int fdt_subnode_offset_namelen(const void *fdt, int parentoffset,
			       const char *name, int namelen);
int fdt_subnode_offset(const void *fdt, int parentoffset, const char *name);

int fdt_path_offset(const void *fdt, const char *path);

const char *fdt_get_name(const void *fdt, int nodeoffset, int *lenp);

const struct fdt_property *fdt_get_property(const void *fdt, int nodeoffset,
					    const char *name, int *lenp);
static inline struct fdt_property *fdt_get_property_w(void *fdt, int nodeoffset,
						      const char *name,
						      int *lenp)
{
	return (struct fdt_property *)(uintptr_t)
		fdt_get_property(fdt, nodeoffset, name, lenp);
}

const void *fdt_getprop(const void *fdt, int nodeoffset,
			const char *name, int *lenp);
static inline void *fdt_getprop_w(void *fdt, int nodeoffset,
				  const char *name, int *lenp)
{
	return (void *)(uintptr_t)fdt_getprop(fdt, nodeoffset, name, lenp);
}

uint32_t fdt_get_phandle(const void *fdt, int nodeoffset);

int fdt_get_path(const void *fdt, int nodeoffset, char *buf, int buflen);

int fdt_supernode_atdepth_offset(const void *fdt, int nodeoffset,
				 int supernodedepth, int *nodedepth);

int fdt_node_depth(const void *fdt, int nodeoffset);

int fdt_parent_offset(const void *fdt, int nodeoffset);

int fdt_node_offset_by_prop_value(const void *fdt, int startoffset,
				  const char *propname,
				  const void *propval, int proplen);

int fdt_node_offset_by_phandle(const void *fdt, uint32_t phandle);

int fdt_node_check_compatible(const void *fdt, int nodeoffset,
			      const char *compatible);

int fdt_node_offset_by_compatible(const void *fdt, int startoffset,
				  const char *compatible);


int fdt_setprop_inplace(void *fdt, int nodeoffset, const char *name,
			const void *val, int len);

static inline int fdt_setprop_inplace_cell(void *fdt, int nodeoffset,
					   const char *name, uint32_t val)
{
	val = cpu_to_fdt32(val);
	return fdt_setprop_inplace(fdt, nodeoffset, name, &val, sizeof(val));
}

int fdt_nop_property(void *fdt, int nodeoffset, const char *name);

int fdt_nop_node(void *fdt, int nodeoffset);


int fdt_create(void *buf, int bufsize);
int fdt_add_reservemap_entry(void *fdt, uint64_t addr, uint64_t size);
int fdt_finish_reservemap(void *fdt);
int fdt_begin_node(void *fdt, const char *name);
int fdt_property(void *fdt, const char *name, const void *val, int len);
static inline int fdt_property_cell(void *fdt, const char *name, uint32_t val)
{
	val = cpu_to_fdt32(val);
	return fdt_property(fdt, name, &val, sizeof(val));
}
#define fdt_property_string(fdt, name, str) \
	fdt_property(fdt, name, str, strlen(str)+1)
int fdt_end_node(void *fdt);
int fdt_finish(void *fdt);


int fdt_open_into(const void *fdt, void *buf, int bufsize);
int fdt_pack(void *fdt);

int fdt_add_mem_rsv(void *fdt, uint64_t address, uint64_t size);

int fdt_del_mem_rsv(void *fdt, int n);

int fdt_set_name(void *fdt, int nodeoffset, const char *name);

int fdt_setprop(void *fdt, int nodeoffset, const char *name,
		const void *val, int len);

static inline int fdt_setprop_cell(void *fdt, int nodeoffset, const char *name,
				   uint32_t val)
{
	val = cpu_to_fdt32(val);
	return fdt_setprop(fdt, nodeoffset, name, &val, sizeof(val));
}

#define fdt_setprop_string(fdt, nodeoffset, name, str) \
	fdt_setprop((fdt), (nodeoffset), (name), (str), strlen(str)+1)

int fdt_delprop(void *fdt, int nodeoffset, const char *name);

int fdt_add_subnode_namelen(void *fdt, int parentoffset,
			    const char *name, int namelen);

int fdt_add_subnode(void *fdt, int parentoffset, const char *name);

int fdt_del_node(void *fdt, int nodeoffset);


const char *fdt_strerror(int errval);

#endif 
