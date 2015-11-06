/*
 * Register cache access API
 *
 * Copyright 2011 Wolfson Microelectronics plc
 *
 * Author: Dimitris Papastamos <dp@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/export.h>
#include <linux/device.h>
#include <trace/events/regmap.h>
#include <linux/bsearch.h>
#include <linux/sort.h>

#include "internal.h"

static const struct regcache_ops *cache_types[] = {
	&regcache_rbtree_ops,
	&regcache_lzo_ops,
};

static int regcache_hw_init(struct regmap *map)
{
	int i, j;
	int ret;
	int count;
	unsigned int val;
	void *tmp_buf;

	if (!map->num_reg_defaults_raw)
		return -EINVAL;

	if (!map->reg_defaults_raw) {
		u32 cache_bypass = map->cache_bypass;
		dev_warn(map->dev, "No cache defaults, reading back from HW\n");

		
		map->cache_bypass = 1;
		tmp_buf = kmalloc(map->cache_size_raw, GFP_KERNEL);
		if (!tmp_buf)
			return -EINVAL;
		ret = regmap_bulk_read(map, 0, tmp_buf,
				       map->num_reg_defaults_raw);
		map->cache_bypass = cache_bypass;
		if (ret < 0) {
			kfree(tmp_buf);
			return ret;
		}
		map->reg_defaults_raw = tmp_buf;
		map->cache_free = 1;
	}

	
	for (count = 0, i = 0; i < map->num_reg_defaults_raw; i++) {
		val = regcache_get_val(map->reg_defaults_raw,
				       i, map->cache_word_size);
		if (regmap_volatile(map, i))
			continue;
		count++;
	}

	map->reg_defaults = kmalloc(count * sizeof(struct reg_default),
				      GFP_KERNEL);
	if (!map->reg_defaults) {
		ret = -ENOMEM;
		goto err_free;
	}

	
	map->num_reg_defaults = count;
	for (i = 0, j = 0; i < map->num_reg_defaults_raw; i++) {
		val = regcache_get_val(map->reg_defaults_raw,
				       i, map->cache_word_size);
		if (regmap_volatile(map, i))
			continue;
		map->reg_defaults[j].reg = i;
		map->reg_defaults[j].def = val;
		j++;
	}

	return 0;

err_free:
	if (map->cache_free)
		kfree(map->reg_defaults_raw);

	return ret;
}

int regcache_init(struct regmap *map, const struct regmap_config *config)
{
	int ret;
	int i;
	void *tmp_buf;

	if (map->cache_type == REGCACHE_NONE) {
		map->cache_bypass = true;
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(cache_types); i++)
		if (cache_types[i]->type == map->cache_type)
			break;

	if (i == ARRAY_SIZE(cache_types)) {
		dev_err(map->dev, "Could not match compress type: %d\n",
			map->cache_type);
		return -EINVAL;
	}

	map->num_reg_defaults = config->num_reg_defaults;
	map->num_reg_defaults_raw = config->num_reg_defaults_raw;
	map->reg_defaults_raw = config->reg_defaults_raw;
	map->cache_word_size = DIV_ROUND_UP(config->val_bits, 8);
	map->cache_size_raw = map->cache_word_size * config->num_reg_defaults_raw;

	map->cache = NULL;
	map->cache_ops = cache_types[i];

	if (!map->cache_ops->read ||
	    !map->cache_ops->write ||
	    !map->cache_ops->name)
		return -EINVAL;

	if (config->reg_defaults) {
		if (!map->num_reg_defaults)
			return -EINVAL;
		tmp_buf = kmemdup(config->reg_defaults, map->num_reg_defaults *
				  sizeof(struct reg_default), GFP_KERNEL);
		if (!tmp_buf)
			return -ENOMEM;
		map->reg_defaults = tmp_buf;
	} else if (map->num_reg_defaults_raw) {
		ret = regcache_hw_init(map);
		if (ret < 0)
			return ret;
	}

	if (!map->max_register)
		map->max_register = map->num_reg_defaults_raw;

	if (map->cache_ops->init) {
		dev_dbg(map->dev, "Initializing %s cache\n",
			map->cache_ops->name);
		ret = map->cache_ops->init(map);
		if (ret)
			goto err_free;
	}
	return 0;

err_free:
	kfree(map->reg_defaults);
	if (map->cache_free)
		kfree(map->reg_defaults_raw);

	return ret;
}

void regcache_exit(struct regmap *map)
{
	if (map->cache_type == REGCACHE_NONE)
		return;

	BUG_ON(!map->cache_ops);

	kfree(map->reg_defaults);
	if (map->cache_free)
		kfree(map->reg_defaults_raw);

	if (map->cache_ops->exit) {
		dev_dbg(map->dev, "Destroying %s cache\n",
			map->cache_ops->name);
		map->cache_ops->exit(map);
	}
}

int regcache_read(struct regmap *map,
		  unsigned int reg, unsigned int *value)
{
	int ret;

	if (map->cache_type == REGCACHE_NONE)
		return -ENOSYS;

	BUG_ON(!map->cache_ops);

	if (!regmap_volatile(map, reg)) {
		ret = map->cache_ops->read(map, reg, value);

		if (ret == 0)
			trace_regmap_reg_read_cache(map->dev, reg, *value);

		return ret;
	}

	return -EINVAL;
}

int regcache_write(struct regmap *map,
		   unsigned int reg, unsigned int value)
{
	if (map->cache_type == REGCACHE_NONE)
		return 0;

	BUG_ON(!map->cache_ops);

	if (!regmap_writeable(map, reg))
		return -EIO;

	if (!regmap_volatile(map, reg))
		return map->cache_ops->write(map, reg, value);

	return 0;
}

int regcache_sync(struct regmap *map)
{
	int ret = 0;
	unsigned int i;
	const char *name;
	unsigned int bypass;

	BUG_ON(!map || !map->cache_ops || !map->cache_ops->sync);

	mutex_lock(&map->lock);
	
	bypass = map->cache_bypass;
	dev_dbg(map->dev, "Syncing %s cache\n",
		map->cache_ops->name);
	name = map->cache_ops->name;
	trace_regcache_sync(map->dev, name, "start");

	if (!map->cache_dirty)
		goto out;

	
	map->cache_bypass = 1;
	for (i = 0; i < map->patch_regs; i++) {
		ret = _regmap_write(map, map->patch[i].reg, map->patch[i].def);
		if (ret != 0) {
			dev_err(map->dev, "Failed to write %x = %x: %d\n",
				map->patch[i].reg, map->patch[i].def, ret);
			goto out;
		}
	}
	map->cache_bypass = 0;

	ret = map->cache_ops->sync(map, 0, map->max_register);

	if (ret == 0)
		map->cache_dirty = false;

out:
	trace_regcache_sync(map->dev, name, "stop");
	
	map->cache_bypass = bypass;
	mutex_unlock(&map->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(regcache_sync);

int regcache_sync_region(struct regmap *map, unsigned int min,
			 unsigned int max)
{
	int ret = 0;
	const char *name;
	unsigned int bypass;

	BUG_ON(!map || !map->cache_ops || !map->cache_ops->sync);

	mutex_lock(&map->lock);

	
	bypass = map->cache_bypass;

	name = map->cache_ops->name;
	dev_dbg(map->dev, "Syncing %s cache from %d-%d\n", name, min, max);

	trace_regcache_sync(map->dev, name, "start region");

	if (!map->cache_dirty)
		goto out;

	ret = map->cache_ops->sync(map, min, max);

out:
	trace_regcache_sync(map->dev, name, "stop region");
	
	map->cache_bypass = bypass;
	mutex_unlock(&map->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(regcache_sync_region);

void regcache_cache_only(struct regmap *map, bool enable)
{
	mutex_lock(&map->lock);
	WARN_ON(map->cache_bypass && enable);
	map->cache_only = enable;
	trace_regmap_cache_only(map->dev, enable);
	mutex_unlock(&map->lock);
}
EXPORT_SYMBOL_GPL(regcache_cache_only);

void regcache_mark_dirty(struct regmap *map)
{
	mutex_lock(&map->lock);
	map->cache_dirty = true;
	mutex_unlock(&map->lock);
}
EXPORT_SYMBOL_GPL(regcache_mark_dirty);

void regcache_cache_bypass(struct regmap *map, bool enable)
{
	mutex_lock(&map->lock);
	WARN_ON(map->cache_only && enable);
	map->cache_bypass = enable;
	trace_regmap_cache_bypass(map->dev, enable);
	mutex_unlock(&map->lock);
}
EXPORT_SYMBOL_GPL(regcache_cache_bypass);

bool regcache_set_val(void *base, unsigned int idx,
		      unsigned int val, unsigned int word_size)
{
	switch (word_size) {
	case 1: {
		u8 *cache = base;
		if (cache[idx] == val)
			return true;
		cache[idx] = val;
		break;
	}
	case 2: {
		u16 *cache = base;
		if (cache[idx] == val)
			return true;
		cache[idx] = val;
		break;
	}
	case 4: {
		u32 *cache = base;
		if (cache[idx] == val)
			return true;
		cache[idx] = val;
		break;
	}
	default:
		BUG();
	}
	return false;
}

unsigned int regcache_get_val(const void *base, unsigned int idx,
			      unsigned int word_size)
{
	if (!base)
		return -EINVAL;

	switch (word_size) {
	case 1: {
		const u8 *cache = base;
		return cache[idx];
	}
	case 2: {
		const u16 *cache = base;
		return cache[idx];
	}
	case 4: {
		const u32 *cache = base;
		return cache[idx];
	}
	default:
		BUG();
	}
	
	return -1;
}

static int regcache_default_cmp(const void *a, const void *b)
{
	const struct reg_default *_a = a;
	const struct reg_default *_b = b;

	return _a->reg - _b->reg;
}

int regcache_lookup_reg(struct regmap *map, unsigned int reg)
{
	struct reg_default key;
	struct reg_default *r;

	key.reg = reg;
	key.def = 0;

	r = bsearch(&key, map->reg_defaults, map->num_reg_defaults,
		    sizeof(struct reg_default), regcache_default_cmp);

	if (r)
		return r - map->reg_defaults;
	else
		return -ENOENT;
}
