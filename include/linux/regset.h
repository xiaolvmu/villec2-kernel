/*
 * User-mode machine state access
 *
 * Copyright (C) 2007 Red Hat, Inc.  All rights reserved.
 *
 * This copyrighted material is made available to anyone wishing to use,
 * modify, copy, or redistribute it subject to the terms and conditions
 * of the GNU General Public License v.2.
 *
 * Red Hat Author: Roland McGrath.
 */

#ifndef _LINUX_REGSET_H
#define _LINUX_REGSET_H	1

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/bug.h>
#include <linux/uaccess.h>
struct task_struct;
struct user_regset;


typedef int user_regset_active_fn(struct task_struct *target,
				  const struct user_regset *regset);

typedef int user_regset_get_fn(struct task_struct *target,
			       const struct user_regset *regset,
			       unsigned int pos, unsigned int count,
			       void *kbuf, void __user *ubuf);

typedef int user_regset_set_fn(struct task_struct *target,
			       const struct user_regset *regset,
			       unsigned int pos, unsigned int count,
			       const void *kbuf, const void __user *ubuf);

typedef int user_regset_writeback_fn(struct task_struct *target,
				     const struct user_regset *regset,
				     int immediate);

struct user_regset {
	user_regset_get_fn		*get;
	user_regset_set_fn		*set;
	user_regset_active_fn		*active;
	user_regset_writeback_fn	*writeback;
	unsigned int			n;
	unsigned int 			size;
	unsigned int 			align;
	unsigned int 			bias;
	unsigned int 			core_note_type;
};

struct user_regset_view {
	const char *name;
	const struct user_regset *regsets;
	unsigned int n;
	u32 e_flags;
	u16 e_machine;
	u8 ei_osabi;
};

const struct user_regset_view *task_user_regset_view(struct task_struct *tsk);



static inline int user_regset_copyout(unsigned int *pos, unsigned int *count,
				      void **kbuf,
				      void __user **ubuf, const void *data,
				      const int start_pos, const int end_pos)
{
	if (*count == 0)
		return 0;
	BUG_ON(*pos < start_pos);
	if (end_pos < 0 || *pos < end_pos) {
		unsigned int copy = (end_pos < 0 ? *count
				     : min(*count, end_pos - *pos));
		data += *pos - start_pos;
		if (*kbuf) {
			memcpy(*kbuf, data, copy);
			*kbuf += copy;
		} else if (__copy_to_user(*ubuf, data, copy))
			return -EFAULT;
		else
			*ubuf += copy;
		*pos += copy;
		*count -= copy;
	}
	return 0;
}

static inline int user_regset_copyin(unsigned int *pos, unsigned int *count,
				     const void **kbuf,
				     const void __user **ubuf, void *data,
				     const int start_pos, const int end_pos)
{
	if (*count == 0)
		return 0;
	BUG_ON(*pos < start_pos);
	if (end_pos < 0 || *pos < end_pos) {
		unsigned int copy = (end_pos < 0 ? *count
				     : min(*count, end_pos - *pos));
		data += *pos - start_pos;
		if (*kbuf) {
			memcpy(data, *kbuf, copy);
			*kbuf += copy;
		} else if (__copy_from_user(data, *ubuf, copy))
			return -EFAULT;
		else
			*ubuf += copy;
		*pos += copy;
		*count -= copy;
	}
	return 0;
}

static inline int user_regset_copyout_zero(unsigned int *pos,
					   unsigned int *count,
					   void **kbuf, void __user **ubuf,
					   const int start_pos,
					   const int end_pos)
{
	if (*count == 0)
		return 0;
	BUG_ON(*pos < start_pos);
	if (end_pos < 0 || *pos < end_pos) {
		unsigned int copy = (end_pos < 0 ? *count
				     : min(*count, end_pos - *pos));
		if (*kbuf) {
			memset(*kbuf, 0, copy);
			*kbuf += copy;
		} else if (__clear_user(*ubuf, copy))
			return -EFAULT;
		else
			*ubuf += copy;
		*pos += copy;
		*count -= copy;
	}
	return 0;
}

static inline int user_regset_copyin_ignore(unsigned int *pos,
					    unsigned int *count,
					    const void **kbuf,
					    const void __user **ubuf,
					    const int start_pos,
					    const int end_pos)
{
	if (*count == 0)
		return 0;
	BUG_ON(*pos < start_pos);
	if (end_pos < 0 || *pos < end_pos) {
		unsigned int copy = (end_pos < 0 ? *count
				     : min(*count, end_pos - *pos));
		if (*kbuf)
			*kbuf += copy;
		else
			*ubuf += copy;
		*pos += copy;
		*count -= copy;
	}
	return 0;
}

static inline int copy_regset_to_user(struct task_struct *target,
				      const struct user_regset_view *view,
				      unsigned int setno,
				      unsigned int offset, unsigned int size,
				      void __user *data)
{
	const struct user_regset *regset = &view->regsets[setno];

	if (!regset->get)
		return -EOPNOTSUPP;

	if (!access_ok(VERIFY_WRITE, data, size))
		return -EFAULT;

	return regset->get(target, regset, offset, size, NULL, data);
}

static inline int copy_regset_from_user(struct task_struct *target,
					const struct user_regset_view *view,
					unsigned int setno,
					unsigned int offset, unsigned int size,
					const void __user *data)
{
	const struct user_regset *regset = &view->regsets[setno];

	if (!regset->set)
		return -EOPNOTSUPP;

	if (!access_ok(VERIFY_READ, data, size))
		return -EFAULT;

	return regset->set(target, regset, offset, size, NULL, data);
}


#endif	
