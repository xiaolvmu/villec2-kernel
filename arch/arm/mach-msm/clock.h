/* arch/arm/mach-msm/clock.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2007-2012, Code Aurora Forum. All rights reserved.
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

#ifndef __ARCH_ARM_MACH_MSM_CLOCK_H
#define __ARCH_ARM_MACH_MSM_CLOCK_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/clkdev.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>

#include <mach/clk.h>

#define MAX_NR_CLKS	300

#define CLKFLAG_INVERT			0x00000001
#define CLKFLAG_NOINVERT		0x00000002
#define CLKFLAG_NONEST			0x00000004
#define CLKFLAG_NORESET			0x00000008
#define CLKFLAG_HWCG			0x00000020
#define CLKFLAG_RETAIN			0x00000040
#define CLKFLAG_NORETAIN		0x00000080
#define CLKFLAG_SKIP_HANDOFF		0x00000100
#define CLKFLAG_MIN			0x00000400
#define CLKFLAG_MAX			0x00000800
#define CLKFLAG_IGNORE			0x10000000	

#define BM(msb, lsb)	(((((uint32_t)-1) << (31-msb)) >> (31-msb+lsb)) << lsb)
#define BVAL(msb, lsb, val)	(((val) << lsb) & BM(msb, lsb))

#define HALT		0	
#define NOCHECK		1	
#define HALT_VOTED	2	
#define ENABLE		3	
#define ENABLE_VOTED	4	
#define DELAY		5	

#define MAX_VDD_LEVELS			4

struct clk_vdd_class {
	const char *class_name;
	int (*set_vdd)(struct clk_vdd_class *v_class, int level);
	int level_votes[MAX_VDD_LEVELS];
	unsigned long cur_level;
	spinlock_t lock;
};

#define DEFINE_VDD_CLASS(_name, _set_vdd) \
	struct clk_vdd_class _name = { \
		.class_name = #_name, \
		.set_vdd = _set_vdd, \
		.cur_level = ARRAY_SIZE(_name.level_votes), \
		.lock = __SPIN_LOCK_UNLOCKED(lock) \
	}

enum handoff {
	HANDOFF_ENABLED_CLK,
	HANDOFF_DISABLED_CLK,
	HANDOFF_UNKNOWN_RATE,
};

struct clk_ops {
	int (*prepare)(struct clk *clk);
	int (*enable)(struct clk *clk);
	void (*disable)(struct clk *clk);
	void (*unprepare)(struct clk *clk);
	void (*enable_hwcg)(struct clk *clk);
	void (*disable_hwcg)(struct clk *clk);
	int (*in_hwcg_mode)(struct clk *clk);
	enum handoff (*handoff)(struct clk *clk);
	int (*reset)(struct clk *clk, enum clk_reset_action action);
	int (*set_rate)(struct clk *clk, unsigned long rate);
	int (*set_max_rate)(struct clk *clk, unsigned long rate);
	int (*set_flags)(struct clk *clk, unsigned flags);
	unsigned long (*get_rate)(struct clk *clk);
	int (*list_rate)(struct clk *clk, unsigned n);
	int (*is_enabled)(struct clk *clk);
	long (*round_rate)(struct clk *clk, unsigned long rate);
	int (*set_parent)(struct clk *clk, struct clk *parent);
	struct clk *(*get_parent)(struct clk *clk);
	bool (*is_local)(struct clk *clk);
};

struct clk {
	uint32_t flags;
	struct clk_ops *ops;
	const char *dbg_name;
	struct clk *depends;
	struct clk_vdd_class *vdd_class;
	unsigned long fmax[MAX_VDD_LEVELS];
	unsigned long rate;

	struct list_head children;
	struct list_head siblings;
	struct list_head enable_list;	

	bool warned;
	unsigned count;
	spinlock_t lock;
	unsigned prepare_count;
	struct mutex prepare_lock;
};

#define CLK_INIT(name) \
	.lock = __SPIN_LOCK_UNLOCKED((name).lock), \
	.prepare_lock = __MUTEX_INITIALIZER((name).prepare_lock), \
	.children = LIST_HEAD_INIT((name).children), \
	.siblings = LIST_HEAD_INIT((name).siblings)

struct clock_init_data {
	struct clk_lookup *table;
	size_t size;
	void (*pre_init)(void);
	void (*post_init)(void);
	int (*late_init)(void);
};

extern struct list_head clk_enable_list;
extern spinlock_t clk_enable_list_lock;
extern int is_xo_src(struct clk *);
extern void clk_ignor_list_add(const char *dev_id, const char *con_id, struct clock_init_data *msm_clock_init_data);

extern struct clock_init_data msm9615_clock_init_data;
extern struct clock_init_data apq8064_clock_init_data;
extern struct clock_init_data deluxe_j_clock_init_data_xb;
extern struct clock_init_data deluxe_ub1_clock_init_data;
extern struct clock_init_data deluxe_u_clock_init_data_xb;
extern struct clock_init_data monarudo_clock_init_data_xd;
extern struct clock_init_data impression_j_clock_init_data_xa;
extern struct clock_init_data m7_clock_init_data_xa;
extern struct clock_init_data m7wl_clock_init_data_xa;

extern struct clock_init_data fsm9xxx_clock_init_data;
extern struct clock_init_data msm7x01a_clock_init_data;
extern struct clock_init_data msm7x27_clock_init_data;
extern struct clock_init_data msm7x27a_clock_init_data;
extern struct clock_init_data msm7x30_clock_init_data;
extern struct clock_init_data msm8960_clock_init_data;
extern struct clock_init_data msm8x60_clock_init_data;
extern struct clock_init_data qds8x50_clock_init_data;
extern struct clock_init_data msm8625_dummy_clock_init_data;
extern struct clock_init_data msm8930_clock_init_data;
extern struct clock_init_data msm8974_clock_init_data;

void msm_clock_init(struct clock_init_data *data);
int vote_vdd_level(struct clk_vdd_class *vdd_class, int level);
int unvote_vdd_level(struct clk_vdd_class *vdd_class, int level);
void keep_dig_voltage_low_in_idle(bool on);

#ifdef CONFIG_DEBUG_FS
int clock_debug_init(struct clock_init_data *data);
int clock_debug_add(struct clk *clock);
void clock_debug_print_enabled(void);
#else
static inline int clock_debug_init(struct clk_init_data *data) { return 0; }
static inline int clock_debug_add(struct clk *clock) { return 0; }
static inline void clock_debug_print_enabled(void) { return; }
#endif

extern struct clk dummy_clk;

#define CLK_DUMMY(clk_name, clk_id, clk_dev, flags) { \
	.con_id = clk_name, \
	.dev_id = clk_dev, \
	.clk = &dummy_clk, \
	}

#define CLK_LOOKUP(con, c, dev) { .con_id = con, .clk = &c, .dev_id = dev }

#endif

