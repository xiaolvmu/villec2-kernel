/*
 *  file.c - part of debugfs, a tiny little debug file system
 *
 *  Copyright (C) 2004 Greg Kroah-Hartman <greg@kroah.com>
 *  Copyright (C) 2004 IBM Inc.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License version
 *	2 as published by the Free Software Foundation.
 *
 *  debugfs is for people to use instead of /proc or /sys.
 *  See Documentation/DocBook/filesystems for more details.
 *
 */

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/pagemap.h>
#include <linux/namei.h>
#include <linux/debugfs.h>
#include <linux/io.h>

static ssize_t default_read_file(struct file *file, char __user *buf,
				 size_t count, loff_t *ppos)
{
	return 0;
}

static ssize_t default_write_file(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	return count;
}

const struct file_operations debugfs_file_operations = {
	.read =		default_read_file,
	.write =	default_write_file,
	.open =		simple_open,
	.llseek =	noop_llseek,
};

static void *debugfs_follow_link(struct dentry *dentry, struct nameidata *nd)
{
	nd_set_link(nd, dentry->d_inode->i_private);
	return NULL;
}

const struct inode_operations debugfs_link_operations = {
	.readlink       = generic_readlink,
	.follow_link    = debugfs_follow_link,
};

static int debugfs_u8_set(void *data, u64 val)
{
	*(u8 *)data = val;
	return 0;
}
static int debugfs_u8_get(void *data, u64 *val)
{
	*val = *(u8 *)data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_u8, debugfs_u8_get, debugfs_u8_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_u8_ro, debugfs_u8_get, NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_u8_wo, NULL, debugfs_u8_set, "%llu\n");

struct dentry *debugfs_create_u8(const char *name, umode_t mode,
				 struct dentry *parent, u8 *value)
{
	
	if (!(mode & S_IWUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_u8_ro);
	
	if (!(mode & S_IRUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_u8_wo);

	return debugfs_create_file(name, mode, parent, value, &fops_u8);
}
EXPORT_SYMBOL_GPL(debugfs_create_u8);

static int debugfs_u16_set(void *data, u64 val)
{
	*(u16 *)data = val;
	return 0;
}
static int debugfs_u16_get(void *data, u64 *val)
{
	*val = *(u16 *)data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_u16, debugfs_u16_get, debugfs_u16_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_u16_ro, debugfs_u16_get, NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_u16_wo, NULL, debugfs_u16_set, "%llu\n");

struct dentry *debugfs_create_u16(const char *name, umode_t mode,
				  struct dentry *parent, u16 *value)
{
	
	if (!(mode & S_IWUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_u16_ro);
	
	if (!(mode & S_IRUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_u16_wo);

	return debugfs_create_file(name, mode, parent, value, &fops_u16);
}
EXPORT_SYMBOL_GPL(debugfs_create_u16);

static int debugfs_u32_set(void *data, u64 val)
{
	*(u32 *)data = val;
	return 0;
}
static int debugfs_u32_get(void *data, u64 *val)
{
	*val = *(u32 *)data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_u32, debugfs_u32_get, debugfs_u32_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_u32_ro, debugfs_u32_get, NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_u32_wo, NULL, debugfs_u32_set, "%llu\n");

struct dentry *debugfs_create_u32(const char *name, umode_t mode,
				 struct dentry *parent, u32 *value)
{
	
	if (!(mode & S_IWUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_u32_ro);
	
	if (!(mode & S_IRUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_u32_wo);

	return debugfs_create_file(name, mode, parent, value, &fops_u32);
}
EXPORT_SYMBOL_GPL(debugfs_create_u32);

static int debugfs_u64_set(void *data, u64 val)
{
	*(u64 *)data = val;
	return 0;
}

static int debugfs_u64_get(void *data, u64 *val)
{
	*val = *(u64 *)data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_u64, debugfs_u64_get, debugfs_u64_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_u64_ro, debugfs_u64_get, NULL, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_u64_wo, NULL, debugfs_u64_set, "%llu\n");

struct dentry *debugfs_create_u64(const char *name, umode_t mode,
				 struct dentry *parent, u64 *value)
{
	
	if (!(mode & S_IWUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_u64_ro);
	
	if (!(mode & S_IRUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_u64_wo);

	return debugfs_create_file(name, mode, parent, value, &fops_u64);
}
EXPORT_SYMBOL_GPL(debugfs_create_u64);

DEFINE_SIMPLE_ATTRIBUTE(fops_x8, debugfs_u8_get, debugfs_u8_set, "0x%02llx\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_x8_ro, debugfs_u8_get, NULL, "0x%02llx\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_x8_wo, NULL, debugfs_u8_set, "0x%02llx\n");

DEFINE_SIMPLE_ATTRIBUTE(fops_x16, debugfs_u16_get, debugfs_u16_set, "0x%04llx\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_x16_ro, debugfs_u16_get, NULL, "0x%04llx\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_x16_wo, NULL, debugfs_u16_set, "0x%04llx\n");

DEFINE_SIMPLE_ATTRIBUTE(fops_x32, debugfs_u32_get, debugfs_u32_set, "0x%08llx\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_x32_ro, debugfs_u32_get, NULL, "0x%08llx\n");
DEFINE_SIMPLE_ATTRIBUTE(fops_x32_wo, NULL, debugfs_u32_set, "0x%08llx\n");

DEFINE_SIMPLE_ATTRIBUTE(fops_x64, debugfs_u64_get, debugfs_u64_set, "0x%016llx\n");


struct dentry *debugfs_create_x8(const char *name, umode_t mode,
				 struct dentry *parent, u8 *value)
{
	
	if (!(mode & S_IWUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_x8_ro);
	
	if (!(mode & S_IRUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_x8_wo);

	return debugfs_create_file(name, mode, parent, value, &fops_x8);
}
EXPORT_SYMBOL_GPL(debugfs_create_x8);

struct dentry *debugfs_create_x16(const char *name, umode_t mode,
				 struct dentry *parent, u16 *value)
{
	
	if (!(mode & S_IWUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_x16_ro);
	
	if (!(mode & S_IRUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_x16_wo);

	return debugfs_create_file(name, mode, parent, value, &fops_x16);
}
EXPORT_SYMBOL_GPL(debugfs_create_x16);

struct dentry *debugfs_create_x32(const char *name, umode_t mode,
				 struct dentry *parent, u32 *value)
{
	
	if (!(mode & S_IWUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_x32_ro);
	
	if (!(mode & S_IRUGO))
		return debugfs_create_file(name, mode, parent, value, &fops_x32_wo);

	return debugfs_create_file(name, mode, parent, value, &fops_x32);
}
EXPORT_SYMBOL_GPL(debugfs_create_x32);

struct dentry *debugfs_create_x64(const char *name, umode_t mode,
				 struct dentry *parent, u64 *value)
{
	return debugfs_create_file(name, mode, parent, value, &fops_x64);
}
EXPORT_SYMBOL_GPL(debugfs_create_x64);


static int debugfs_size_t_set(void *data, u64 val)
{
	*(size_t *)data = val;
	return 0;
}
static int debugfs_size_t_get(void *data, u64 *val)
{
	*val = *(size_t *)data;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(fops_size_t, debugfs_size_t_get, debugfs_size_t_set,
			"%llu\n");	

struct dentry *debugfs_create_size_t(const char *name, umode_t mode,
				     struct dentry *parent, size_t *value)
{
	return debugfs_create_file(name, mode, parent, value, &fops_size_t);
}
EXPORT_SYMBOL_GPL(debugfs_create_size_t);


static ssize_t read_file_bool(struct file *file, char __user *user_buf,
			      size_t count, loff_t *ppos)
{
	char buf[3];
	u32 *val = file->private_data;
	
	if (*val)
		buf[0] = 'Y';
	else
		buf[0] = 'N';
	buf[1] = '\n';
	buf[2] = 0x00;
	return simple_read_from_buffer(user_buf, count, ppos, buf, 2);
}

static ssize_t write_file_bool(struct file *file, const char __user *user_buf,
			       size_t count, loff_t *ppos)
{
	char buf[32];
	size_t buf_size;
	bool bv;
	u32 *val = file->private_data;

	buf_size = min(count, (sizeof(buf)-1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	if (strtobool(buf, &bv) == 0)
		*val = bv;

	return count;
}

static const struct file_operations fops_bool = {
	.read =		read_file_bool,
	.write =	write_file_bool,
	.open =		simple_open,
	.llseek =	default_llseek,
};

struct dentry *debugfs_create_bool(const char *name, umode_t mode,
				   struct dentry *parent, u32 *value)
{
	return debugfs_create_file(name, mode, parent, value, &fops_bool);
}
EXPORT_SYMBOL_GPL(debugfs_create_bool);

static ssize_t read_file_blob(struct file *file, char __user *user_buf,
			      size_t count, loff_t *ppos)
{
	struct debugfs_blob_wrapper *blob = file->private_data;
	return simple_read_from_buffer(user_buf, count, ppos, blob->data,
			blob->size);
}

static const struct file_operations fops_blob = {
	.read =		read_file_blob,
	.open =		simple_open,
	.llseek =	default_llseek,
};

struct dentry *debugfs_create_blob(const char *name, umode_t mode,
				   struct dentry *parent,
				   struct debugfs_blob_wrapper *blob)
{
	return debugfs_create_file(name, mode, parent, blob, &fops_blob);
}
EXPORT_SYMBOL_GPL(debugfs_create_blob);

#ifdef CONFIG_HAS_IOMEM


int debugfs_print_regs32(struct seq_file *s, const struct debugfs_reg32 *regs,
			   int nregs, void __iomem *base, char *prefix)
{
	int i, ret = 0;

	for (i = 0; i < nregs; i++, regs++) {
		if (prefix)
			ret += seq_printf(s, "%s", prefix);
		ret += seq_printf(s, "%s = 0x%08x\n", regs->name,
				  readl(base + regs->offset));
	}
	return ret;
}
EXPORT_SYMBOL_GPL(debugfs_print_regs32);

static int debugfs_show_regset32(struct seq_file *s, void *data)
{
	struct debugfs_regset32 *regset = s->private;

	debugfs_print_regs32(s, regset->regs, regset->nregs, regset->base, "");
	return 0;
}

static int debugfs_open_regset32(struct inode *inode, struct file *file)
{
	return single_open(file, debugfs_show_regset32, inode->i_private);
}

static const struct file_operations fops_regset32 = {
	.open =		debugfs_open_regset32,
	.read =		seq_read,
	.llseek =	seq_lseek,
	.release =	single_release,
};

struct dentry *debugfs_create_regset32(const char *name, umode_t mode,
				       struct dentry *parent,
				       struct debugfs_regset32 *regset)
{
	return debugfs_create_file(name, mode, parent, regset, &fops_regset32);
}
EXPORT_SYMBOL_GPL(debugfs_create_regset32);

#endif 
