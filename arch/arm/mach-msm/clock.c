/* arch/arm/mach-msm/clock.c
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/list.h>
#include <trace/events/power.h>

#include "clock.h"

struct handoff_clk {
	struct list_head list;
	struct clk *clk;
};
static LIST_HEAD(handoff_list);

static int find_vdd_level(struct clk *clk, unsigned long rate)
{
	int level;

	for (level = 0; level < ARRAY_SIZE(clk->fmax); level++)
		if (rate <= clk->fmax[level])
			break;

	if (level == ARRAY_SIZE(clk->fmax)) {
		pr_err("Rate %lu for %s is greater than highest Fmax\n", rate,
			clk->dbg_name);
		return -EINVAL;
	}

	return level;
}

static int update_vdd(struct clk_vdd_class *vdd_class)
{
	int level, rc;

	for (level = ARRAY_SIZE(vdd_class->level_votes)-1; level > 0; level--)
		if (vdd_class->level_votes[level])
			break;

	if (level == vdd_class->cur_level)
		return 0;

	rc = vdd_class->set_vdd(vdd_class, level);
	if (!rc)
		vdd_class->cur_level = level;

	return rc;
}

int vote_vdd_level(struct clk_vdd_class *vdd_class, int level)
{
	unsigned long flags;
	int rc;

	spin_lock_irqsave(&vdd_class->lock, flags);
	vdd_class->level_votes[level]++;
	rc = update_vdd(vdd_class);
	if (rc)
		vdd_class->level_votes[level]--;
	spin_unlock_irqrestore(&vdd_class->lock, flags);

	return rc;
}

int unvote_vdd_level(struct clk_vdd_class *vdd_class, int level)
{
	unsigned long flags;
	int rc = 0;

	spin_lock_irqsave(&vdd_class->lock, flags);
	if (WARN(!vdd_class->level_votes[level],
			"Reference counts are incorrect for %s level %d\n",
			vdd_class->class_name, level))
		goto out;
	vdd_class->level_votes[level]--;
	rc = update_vdd(vdd_class);
	if (rc)
		vdd_class->level_votes[level]++;
out:
	spin_unlock_irqrestore(&vdd_class->lock, flags);
	return rc;
}

static int vote_rate_vdd(struct clk *clk, unsigned long rate)
{
	int level;

	if (!clk->vdd_class)
		return 0;

	level = find_vdd_level(clk, rate);
	if (level < 0)
		return level;

	return vote_vdd_level(clk->vdd_class, level);
}

static void unvote_rate_vdd(struct clk *clk, unsigned long rate)
{
	int level;

	if (!clk->vdd_class)
		return;

	level = find_vdd_level(clk, rate);
	if (level < 0)
		return;

	unvote_vdd_level(clk->vdd_class, level);
}

static bool is_rate_valid(struct clk *clk, unsigned long rate)
{
	int level;

	if (!clk->vdd_class)
		return true;

	level = find_vdd_level(clk, rate);
	return level >= 0;
}

int clk_prepare(struct clk *clk)
{
	int ret = 0;
	struct clk *parent;

	if (!clk)
		return 0;
	if (IS_ERR(clk))
		return -EINVAL;

	mutex_lock(&clk->prepare_lock);
	if (clk->prepare_count == 0) {
		parent = clk_get_parent(clk);

		ret = clk_prepare(parent);
		if (ret)
			goto out;
		ret = clk_prepare(clk->depends);
		if (ret)
			goto err_prepare_depends;

		if (clk->ops->prepare)
			ret = clk->ops->prepare(clk);
		if (ret)
			goto err_prepare_clock;
	}
	clk->prepare_count++;
out:
	mutex_unlock(&clk->prepare_lock);
	return ret;
err_prepare_clock:
	clk_unprepare(clk->depends);
err_prepare_depends:
	clk_unprepare(parent);
	goto out;
}
EXPORT_SYMBOL(clk_prepare);
LIST_HEAD(clk_enable_list);
DEFINE_SPINLOCK(clk_enable_list_lock);
int clk_enable(struct clk *clk)
{
	int ret = 0;
	unsigned long flags;
	struct clk *parent;

	if (!clk)
		return 0;
	if (IS_ERR(clk))
		return -EINVAL;

	spin_lock_irqsave(&clk->lock, flags);
	if (WARN(!clk->warned && !clk->prepare_count,
				"%s: Don't call enable on unprepared clocks\n",
				clk->dbg_name))
		clk->warned = true;
	if (clk->count == 0) {
		parent = clk_get_parent(clk);
		if (!(clk->flags&CLKFLAG_IGNORE)) {
			ret = clk_enable(parent);
			if (ret)
				goto err_enable_parent;
			ret = clk_enable(clk->depends);
			if (ret)
				goto err_enable_depends;
		}

		ret = vote_rate_vdd(clk, clk->rate);
		if (ret)
			goto err_vote_vdd;
		trace_clock_enable(clk->dbg_name, 1, smp_processor_id());
		if (clk->ops->enable)
			ret = clk->ops->enable(clk);
		if (ret)
			goto err_enable_clock;
		
		spin_lock(&clk_enable_list_lock);
		if (!(clk->flags&CLKFLAG_IGNORE))
			list_add(&clk->enable_list, &clk_enable_list);	
		spin_unlock(&clk_enable_list_lock);
	}
	clk->count++;
	spin_unlock_irqrestore(&clk->lock, flags);

	return 0;

err_enable_clock:
	unvote_rate_vdd(clk, clk->rate);
err_vote_vdd:
	clk_disable(clk->depends);
err_enable_depends:
	clk_disable(parent);
err_enable_parent:
	spin_unlock_irqrestore(&clk->lock, flags);
	return ret;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;

	if (IS_ERR_OR_NULL(clk))
		return;

	spin_lock_irqsave(&clk->lock, flags);
	if (WARN(!clk->warned && !clk->prepare_count,
				"%s: Never called prepare or calling disable "
				"after unprepare\n",
				clk->dbg_name))
		clk->warned = true;
	if (WARN(clk->count == 0, "%s is unbalanced", clk->dbg_name))
		goto out;
	if (clk->count == 1) {
		struct clk *parent = clk_get_parent(clk);

		trace_clock_disable(clk->dbg_name, 0, smp_processor_id());
		if (clk->ops->disable)
			clk->ops->disable(clk);
		unvote_rate_vdd(clk, clk->rate);

		
		if (!(clk->flags&CLKFLAG_IGNORE)) {
			clk_disable(clk->depends);
			clk_disable(parent);
			spin_lock(&clk_enable_list_lock);
			list_del(&clk->enable_list);
			spin_unlock(&clk_enable_list_lock);
		}
	}
	clk->count--;
out:
	spin_unlock_irqrestore(&clk->lock, flags);
}
EXPORT_SYMBOL(clk_disable);

void clk_unprepare(struct clk *clk)
{
	if (IS_ERR_OR_NULL(clk))
		return;

	mutex_lock(&clk->prepare_lock);
	if (!clk->prepare_count) {
		if (WARN(!clk->warned, "%s is unbalanced (prepare)",
				clk->dbg_name))
			clk->warned = true;
		goto out;
	}
	if (clk->prepare_count == 1) {
		struct clk *parent = clk_get_parent(clk);

		if (WARN(!clk->warned && clk->count,
			"%s: Don't call unprepare when the clock is enabled\n",
				clk->dbg_name))
			clk->warned = true;

		if (clk->ops->unprepare)
			clk->ops->unprepare(clk);
		clk_unprepare(clk->depends);
		clk_unprepare(parent);
	}
	clk->prepare_count--;
out:
	mutex_unlock(&clk->prepare_lock);
}
EXPORT_SYMBOL(clk_unprepare);

int clk_reset(struct clk *clk, enum clk_reset_action action)
{
	if (IS_ERR_OR_NULL(clk))
		return -EINVAL;

	if (!clk->ops->reset)
		return -ENOSYS;

	return clk->ops->reset(clk, action);
}
EXPORT_SYMBOL(clk_reset);

unsigned long clk_get_rate(struct clk *clk)
{
	if (IS_ERR_OR_NULL(clk))
		return 0;

	if (!clk->ops->get_rate)
		return clk->rate;

	return clk->ops->get_rate(clk);
}
EXPORT_SYMBOL(clk_get_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long start_rate, flags;
	int rc = 0;

	if (IS_ERR_OR_NULL(clk))
		return -EINVAL;

	if (!clk->ops->set_rate)
		return -ENOSYS;

	spin_lock_irqsave(&clk->lock, flags);

	
	if (clk->rate == rate)
		goto out;

	trace_clock_set_rate(clk->dbg_name, rate, smp_processor_id());
	if (clk->count) {
		start_rate = clk->rate;
		
		rc = vote_rate_vdd(clk, rate);
		if (rc)
			goto out;
		rc = clk->ops->set_rate(clk, rate);
		if (rc)
			goto err_set_rate;
		
		unvote_rate_vdd(clk, start_rate);
	} else if (is_rate_valid(clk, rate)) {
		rc = clk->ops->set_rate(clk, rate);
	} else {
		rc = -EINVAL;
	}

	if (!rc)
		clk->rate = rate;
out:
	spin_unlock_irqrestore(&clk->lock, flags);
	return rc;

err_set_rate:
	unvote_rate_vdd(clk, rate);
	goto out;
}
EXPORT_SYMBOL(clk_set_rate);

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (IS_ERR_OR_NULL(clk))
		return -EINVAL;

	if (!clk->ops->round_rate)
		return -ENOSYS;

	return clk->ops->round_rate(clk, rate);
}
EXPORT_SYMBOL(clk_round_rate);

int clk_set_max_rate(struct clk *clk, unsigned long rate)
{
	if (IS_ERR_OR_NULL(clk))
		return -EINVAL;

	if (!clk->ops->set_max_rate)
		return -ENOSYS;

	return clk->ops->set_max_rate(clk, rate);
}
EXPORT_SYMBOL(clk_set_max_rate);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	if (!clk->ops->set_parent)
		return 0;

	return clk->ops->set_parent(clk, parent);
}
EXPORT_SYMBOL(clk_set_parent);

struct clk *clk_get_parent(struct clk *clk)
{
	if (IS_ERR_OR_NULL(clk))
		return NULL;

	if (!clk->ops->get_parent)
		return NULL;

	return clk->ops->get_parent(clk);
}
EXPORT_SYMBOL(clk_get_parent);

int clk_set_flags(struct clk *clk, unsigned long flags)
{
	if (IS_ERR_OR_NULL(clk))
		return -EINVAL;
	if (!clk->ops->set_flags)
		return -ENOSYS;

	return clk->ops->set_flags(clk, flags);
}
EXPORT_SYMBOL(clk_set_flags);

static struct clock_init_data __initdata *clk_init_data;

static enum handoff __init __handoff_clk(struct clk *clk)
{
	enum handoff ret;
	struct handoff_clk *h;
	unsigned long rate;
	int err = 0;

	if (clk == NULL || clk->count)
		return HANDOFF_ENABLED_CLK;

	
	if (!clk->ops->handoff || (clk->flags & CLKFLAG_SKIP_HANDOFF))
		return HANDOFF_DISABLED_CLK;

	ret = clk->ops->handoff(clk);
	if (ret == HANDOFF_ENABLED_CLK) {
		ret = __handoff_clk(clk_get_parent(clk));
		if (ret == HANDOFF_ENABLED_CLK) {
			h = kmalloc(sizeof(*h), GFP_KERNEL);
			if (!h) {
				err = -ENOMEM;
				goto out;
			}
			err = clk_prepare_enable(clk);
			if (err)
				goto out;
			rate = clk_get_rate(clk);
			if (rate)
				pr_debug("%s rate=%lu\n", clk->dbg_name, rate);
			h->clk = clk;
			list_add_tail(&h->list, &handoff_list);
		}
	}
out:
	if (err) {
		pr_err("%s handoff failed (%d)\n", clk->dbg_name, err);
		kfree(h);
		ret = HANDOFF_DISABLED_CLK;
	}
	return ret;
}

void __init msm_clock_init(struct clock_init_data *data)
{
	unsigned n;
	struct clk_lookup *clock_tbl;
	size_t num_clocks;
	struct clk *clk;

	clk_init_data = data;
	if (clk_init_data->pre_init)
		clk_init_data->pre_init();

	clock_tbl = data->table;
	num_clocks = data->size;

	for (n = 0; n < num_clocks; n++) {
		struct clk *parent;
		clk = clock_tbl[n].clk;
		parent = clk_get_parent(clk);
		if (parent && list_empty(&clk->siblings))
			list_add(&clk->siblings, &parent->children);
	}

	for (n = 0; n < num_clocks; n++)
		__handoff_clk(clock_tbl[n].clk);

	clkdev_add_table(clock_tbl, num_clocks);

	if (clk_init_data->post_init)
		clk_init_data->post_init();
}

static int __init clock_late_init(void)
{
	struct handoff_clk *h, *h_temp;
	int n, ret = 0;

	clock_debug_init(clk_init_data);
	for (n = 0; n < clk_init_data->size; n++)
		clock_debug_add(clk_init_data->table[n].clk);

	pr_info("%s: Removing enables held for handed-off clocks\n", __func__);
	list_for_each_entry_safe(h, h_temp, &handoff_list, list) {
		clk_disable_unprepare(h->clk);
		list_del(&h->list);
		kfree(h);
	}

	if (clk_init_data->late_init)
		ret = clk_init_data->late_init();
	return ret;
}
late_initcall(clock_late_init);
