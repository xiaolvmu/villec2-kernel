/* linux/include/asm-arm/arch-msm/msm_smd.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2009-2012, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MSM_SMD_H
#define __ASM_ARCH_MSM_SMD_H

#include <linux/io.h>
#include <mach/msm_smsm.h>

typedef struct smd_channel smd_channel_t;

#define SMD_MAX_CH_NAME_LEN 20 

#define SMD_EVENT_DATA 1
#define SMD_EVENT_OPEN 2
#define SMD_EVENT_CLOSE 3
#define SMD_EVENT_STATUS 4
#define SMD_EVENT_REOPEN_READY 5

enum {
	SMD_APPS = SMSM_APPS,
	SMD_MODEM = SMSM_MODEM,
	SMD_Q6 = SMSM_Q6,
	SMD_WCNSS = SMSM_WCNSS,
	SMD_DSPS = SMSM_DSPS,
	SMD_MODEM_Q6_FW,
	SMD_RPM,
	NUM_SMD_SUBSYSTEMS,
};

enum {
	SMD_APPS_MODEM = 0,
	SMD_APPS_QDSP,
	SMD_MODEM_QDSP,
	SMD_APPS_DSPS,
	SMD_MODEM_DSPS,
	SMD_QDSP_DSPS,
	SMD_APPS_WCNSS,
	SMD_MODEM_WCNSS,
	SMD_QDSP_WCNSS,
	SMD_DSPS_WCNSS,
	SMD_APPS_Q6FW,
	SMD_MODEM_Q6FW,
	SMD_QDSP_Q6FW,
	SMD_DSPS_Q6FW,
	SMD_WCNSS_Q6FW,
	SMD_APPS_RPM,
	SMD_MODEM_RPM,
	SMD_QDSP_RPM,
	SMD_WCNSS_RPM,
	SMD_NUM_TYPE,
	SMD_LOOPBACK_TYPE = 100,

};


struct smd_irq_config {
	
	const char *irq_name;
	unsigned long flags;
	int irq_id;
	const char *device_name;
	const void *dev_id;

	
	uint32_t out_bit_pos;
	void __iomem *out_base;
	uint32_t out_offset;
};

struct smd_subsystem_config {
	unsigned irq_config_id;
	const char *subsys_name;
	int edge;

	struct smd_irq_config smd_int;
	struct smd_irq_config smsm_int;

};

struct smd_subsystem_restart_config {
	int disable_smsm_reset_handshake;
};

struct smd_smem_regions {
	void *phys_addr;
	unsigned size;
};

struct smd_platform {
	uint32_t num_ss_configs;
	struct smd_subsystem_config *smd_ss_configs;
	struct smd_subsystem_restart_config *smd_ssr_config;
	uint32_t num_smem_areas;
	struct smd_smem_regions *smd_smem_areas;
};

#ifdef CONFIG_MSM_SMD
int smd_open(const char *name, smd_channel_t **ch, void *priv,
	     void (*notify)(void *priv, unsigned event));

int smd_close(smd_channel_t *ch);

int smd_read(smd_channel_t *ch, void *data, int len);
int smd_read_from_cb(smd_channel_t *ch, void *data, int len);
int smd_read_user_buffer(smd_channel_t *ch, void *data, int len);

int smd_write(smd_channel_t *ch, const void *data, int len);
int smd_write_user_buffer(smd_channel_t *ch, const void *data, int len);

int smd_write_avail(smd_channel_t *ch);
int smd_read_avail(smd_channel_t *ch);

int smd_cur_packet_size(smd_channel_t *ch);


#if 0
int smd_wait_until_readable(smd_channel_t *ch, int bytes);
int smd_wait_until_writable(smd_channel_t *ch, int bytes);
#endif

int smd_tiocmget(smd_channel_t *ch);
int smd_tiocmset(smd_channel_t *ch, unsigned int set, unsigned int clear);
int
smd_tiocmset_from_cb(smd_channel_t *ch, unsigned int set, unsigned int clear);
int smd_named_open_on_edge(const char *name, uint32_t edge, smd_channel_t **_ch,
			   void *priv, void (*notify)(void *, unsigned));

void smd_enable_read_intr(smd_channel_t *ch);

void smd_disable_read_intr(smd_channel_t *ch);

int smd_write_start(smd_channel_t *ch, int len);

int smd_write_segment(smd_channel_t *ch, void *data, int len, int user_buf);

int smd_write_end(smd_channel_t *ch);

const char *smd_edge_to_subsystem(uint32_t type);

const char *smd_pid_to_subsystem(uint32_t pid);

int smd_is_pkt_avail(smd_channel_t *ch);

int __init msm_smd_init(void);

#else

static inline int smd_open(const char *name, smd_channel_t **ch, void *priv,
	     void (*notify)(void *priv, unsigned event))
{
	return -ENODEV;
}

static inline int smd_close(smd_channel_t *ch)
{
	return -ENODEV;
}

static inline int smd_read(smd_channel_t *ch, void *data, int len)
{
	return -ENODEV;
}

static inline int smd_read_from_cb(smd_channel_t *ch, void *data, int len)
{
	return -ENODEV;
}

static inline int smd_read_user_buffer(smd_channel_t *ch, void *data, int len)
{
	return -ENODEV;
}

static inline int smd_write(smd_channel_t *ch, const void *data, int len)
{
	return -ENODEV;
}

static inline int
smd_write_user_buffer(smd_channel_t *ch, const void *data, int len)
{
	return -ENODEV;
}

static inline int smd_write_avail(smd_channel_t *ch)
{
	return -ENODEV;
}

static inline int smd_read_avail(smd_channel_t *ch)
{
	return -ENODEV;
}

static inline int smd_cur_packet_size(smd_channel_t *ch)
{
	return -ENODEV;
}

static inline int smd_tiocmget(smd_channel_t *ch)
{
	return -ENODEV;
}

static inline int
smd_tiocmset(smd_channel_t *ch, unsigned int set, unsigned int clear)
{
	return -ENODEV;
}

static inline int
smd_tiocmset_from_cb(smd_channel_t *ch, unsigned int set, unsigned int clear)
{
	return -ENODEV;
}

static inline int
smd_named_open_on_edge(const char *name, uint32_t edge, smd_channel_t **_ch,
			   void *priv, void (*notify)(void *, unsigned))
{
	return -ENODEV;
}

static inline void smd_enable_read_intr(smd_channel_t *ch)
{
}

static inline void smd_disable_read_intr(smd_channel_t *ch)
{
}

static inline int smd_write_start(smd_channel_t *ch, int len)
{
	return -ENODEV;
}

static inline int
smd_write_segment(smd_channel_t *ch, void *data, int len, int user_buf)
{
	return -ENODEV;
}

static inline int smd_write_end(smd_channel_t *ch)
{
	return -ENODEV;
}

static inline const char *smd_edge_to_subsystem(uint32_t type)
{
	return NULL;
}

static inline const char *smd_pid_to_subsystem(uint32_t pid)
{
	return NULL;
}

static inline int smd_is_pkt_avail(smd_channel_t *ch)
{
	return -ENODEV;
}

static inline int __init msm_smd_init(void)
{
	return 0;
}
#endif

#endif
