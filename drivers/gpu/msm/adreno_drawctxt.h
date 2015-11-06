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
#ifndef __ADRENO_DRAWCTXT_H
#define __ADRENO_DRAWCTXT_H

#include "adreno_pm4types.h"
#include "a2xx_reg.h"


#define CTXT_FLAGS_NOT_IN_USE		0x00000000
#define CTXT_FLAGS_IN_USE		0x00000001

#define CTXT_FLAGS_STATE_SHADOW		0x00000010

#define CTXT_FLAGS_GMEM_SHADOW		0x00000100
#define CTXT_FLAGS_GMEM_SAVE		0x00000200
#define CTXT_FLAGS_GMEM_RESTORE		0x00000400
#define CTXT_FLAGS_PREAMBLE		0x00000800
#define CTXT_FLAGS_SHADER_SAVE		0x00002000
#define CTXT_FLAGS_SHADER_RESTORE	0x00004000
#define CTXT_FLAGS_GPU_HANG		0x00008000
#define CTXT_FLAGS_NOGMEMALLOC          0x00010000
#define CTXT_FLAGS_TRASHSTATE		0x00020000
#define CTXT_FLAGS_PER_CONTEXT_TS	0x00040000
#define CTXT_FLAGS_GPU_HANG_RECOVERED	0x00008000

struct kgsl_device;
struct adreno_device;
struct kgsl_device_private;
struct kgsl_context;

struct gmem_shadow_t {
	struct kgsl_memdesc gmemshadow;	


	enum COLORFORMATX format; 
	unsigned int size;	
	unsigned int width;	
	unsigned int height;	
	unsigned int pitch;	
	unsigned int gmem_pitch;	
	unsigned int *gmem_save_commands;    
	unsigned int *gmem_restore_commands; 
	unsigned int gmem_save[3];
	unsigned int gmem_restore[3];
	struct kgsl_memdesc quad_vertices;
	struct kgsl_memdesc quad_texcoords;
	struct kgsl_memdesc quad_vertices_restore;
};

struct adreno_context {
	unsigned int id;
	uint32_t flags;
	struct kgsl_pagetable *pagetable;
	struct kgsl_memdesc gpustate;
	unsigned int reg_restore[3];
	unsigned int shader_save[3];
	unsigned int shader_restore[3];

	
	struct gmem_shadow_t context_gmem_shadow;

	
	unsigned int reg_save[3];
	unsigned int shader_fixup[3];
	unsigned int chicken_restore[3];
	unsigned int bin_base_offset;

	
	unsigned int regconstant_save[3];
	unsigned int constant_restore[3];
	unsigned int hlsqcontrol_restore[3];
	unsigned int save_fixup[3];
	unsigned int restore_fixup[3];
	struct kgsl_memdesc shader_load_commands[2];
	struct kgsl_memdesc shader_save_commands[4];
	struct kgsl_memdesc constant_save_commands[3];
	struct kgsl_memdesc constant_load_commands[3];
	struct kgsl_memdesc cond_execs[4];
	struct kgsl_memdesc hlsqcontrol_restore_commands[1];
};

int adreno_drawctxt_create(struct kgsl_device *device,
			struct kgsl_pagetable *pagetable,
			struct kgsl_context *context,
			uint32_t flags);

void adreno_drawctxt_destroy(struct kgsl_device *device,
			  struct kgsl_context *context);

void adreno_drawctxt_switch(struct adreno_device *adreno_dev,
				struct adreno_context *drawctxt,
				unsigned int flags);
void adreno_drawctxt_set_bin_base_offset(struct kgsl_device *device,
					struct kgsl_context *context,
					unsigned int offset);


void build_quad_vtxbuff(struct adreno_context *drawctxt,
		struct gmem_shadow_t *shadow, unsigned int **incmd);

unsigned int uint2float(unsigned int);

static inline unsigned int virt2gpu(unsigned int *cmd,
				    struct kgsl_memdesc *memdesc)
{
	return memdesc->gpuaddr + ((char *) cmd - (char *) memdesc->hostptr);
}

static inline void create_ib1(struct adreno_context *drawctxt,
			      unsigned int *cmd,
			      unsigned int *start,
			      unsigned int *end)
{
	cmd[0] = CP_HDR_INDIRECT_BUFFER_PFD;
	cmd[1] = virt2gpu(start, &drawctxt->gpustate);
	cmd[2] = end - start;
}


static inline unsigned int *reg_range(unsigned int *cmd, unsigned int start,
	unsigned int end)
{
	*cmd++ = CP_REG(start);		
	*cmd++ = end - start + 1;	
	return cmd;
}

static inline void calc_gmemsize(struct gmem_shadow_t *shadow, int gmem_size)
{
	int w = 64, h = 64;

	shadow->format = COLORX_8_8_8_8;

	
	gmem_size = (gmem_size + 3) / 4;

	while ((w * h) < gmem_size) {
		if (w < h)
			w *= 2;
		else
			h *= 2;
	}

	shadow->pitch = shadow->width = w;
	shadow->height = h;
	shadow->gmem_pitch = shadow->pitch;
	shadow->size = shadow->pitch * shadow->height * 4;
}

#endif  
