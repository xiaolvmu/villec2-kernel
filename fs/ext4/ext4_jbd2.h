/*
 * ext4_jbd2.h
 *
 * Written by Stephen C. Tweedie <sct@redhat.com>, 1999
 *
 * Copyright 1998--1999 Red Hat corp --- All Rights Reserved
 *
 * This file is part of the Linux kernel and is made available under
 * the terms of the GNU General Public License, version 2, or at your
 * option, any later version, incorporated herein by reference.
 *
 * Ext4-specific journaling extensions.
 */

#ifndef _EXT4_JBD2_H
#define _EXT4_JBD2_H

#include <linux/fs.h>
#include <linux/jbd2.h>
#include "ext4.h"

#define EXT4_JOURNAL(inode)	(EXT4_SB((inode)->i_sb)->s_journal)


#define EXT4_SINGLEDATA_TRANS_BLOCKS(sb)				\
	(EXT4_HAS_INCOMPAT_FEATURE(sb, EXT4_FEATURE_INCOMPAT_EXTENTS)   \
	 ? 27U : 8U)


#define EXT4_XATTR_TRANS_BLOCKS		6U


#define EXT4_DATA_TRANS_BLOCKS(sb)	(EXT4_SINGLEDATA_TRANS_BLOCKS(sb) + \
					 EXT4_XATTR_TRANS_BLOCKS - 2 + \
					 EXT4_MAXQUOTAS_TRANS_BLOCKS(sb))

#define EXT4_META_TRANS_BLOCKS(sb)	(EXT4_XATTR_TRANS_BLOCKS + \
					EXT4_MAXQUOTAS_TRANS_BLOCKS(sb))


#define EXT4_DELETE_TRANS_BLOCKS(sb)	(2 * EXT4_DATA_TRANS_BLOCKS(sb) + 64)


#define EXT4_MAX_TRANS_DATA		64U


#define EXT4_RESERVE_TRANS_BLOCKS	12U

#define EXT4_INDEX_EXTRA_TRANS_BLOCKS	8

#ifdef CONFIG_QUOTA
#define EXT4_QUOTA_TRANS_BLOCKS(sb) (test_opt(sb, QUOTA) ? 1 : 0)
#define EXT4_QUOTA_INIT_BLOCKS(sb) (test_opt(sb, QUOTA) ? (DQUOT_INIT_ALLOC*\
		(EXT4_SINGLEDATA_TRANS_BLOCKS(sb)-3)+3+DQUOT_INIT_REWRITE) : 0)

#define EXT4_QUOTA_DEL_BLOCKS(sb) (test_opt(sb, QUOTA) ? (DQUOT_DEL_ALLOC*\
		(EXT4_SINGLEDATA_TRANS_BLOCKS(sb)-3)+3+DQUOT_DEL_REWRITE) : 0)
#else
#define EXT4_QUOTA_TRANS_BLOCKS(sb) 0
#define EXT4_QUOTA_INIT_BLOCKS(sb) 0
#define EXT4_QUOTA_DEL_BLOCKS(sb) 0
#endif
#define EXT4_MAXQUOTAS_TRANS_BLOCKS(sb) (MAXQUOTAS*EXT4_QUOTA_TRANS_BLOCKS(sb))
#define EXT4_MAXQUOTAS_INIT_BLOCKS(sb) (MAXQUOTAS*EXT4_QUOTA_INIT_BLOCKS(sb))
#define EXT4_MAXQUOTAS_DEL_BLOCKS(sb) (MAXQUOTAS*EXT4_QUOTA_DEL_BLOCKS(sb))

struct ext4_journal_cb_entry {
	
	struct list_head jce_list;

	
	void (*jce_func)(struct super_block *sb,
			 struct ext4_journal_cb_entry *jce, int error);

	
};

static inline void ext4_journal_callback_add(handle_t *handle,
			void (*func)(struct super_block *sb,
				     struct ext4_journal_cb_entry *jce,
				     int rc),
			struct ext4_journal_cb_entry *jce)
{
	struct ext4_sb_info *sbi =
			EXT4_SB(handle->h_transaction->t_journal->j_private);

	
	jce->jce_func = func;
	spin_lock(&sbi->s_md_lock);
	list_add_tail(&jce->jce_list, &handle->h_transaction->t_private_list);
	spin_unlock(&sbi->s_md_lock);
}

static inline void ext4_journal_callback_del(handle_t *handle,
					     struct ext4_journal_cb_entry *jce)
{
	struct ext4_sb_info *sbi =
			EXT4_SB(handle->h_transaction->t_journal->j_private);

	spin_lock(&sbi->s_md_lock);
	list_del_init(&jce->jce_list);
	spin_unlock(&sbi->s_md_lock);
}

int
ext4_mark_iloc_dirty(handle_t *handle,
		     struct inode *inode,
		     struct ext4_iloc *iloc);


int ext4_reserve_inode_write(handle_t *handle, struct inode *inode,
			struct ext4_iloc *iloc);

int ext4_mark_inode_dirty(handle_t *handle, struct inode *inode);

void ext4_journal_abort_handle(const char *caller, unsigned int line,
			       const char *err_fn,
		struct buffer_head *bh, handle_t *handle, int err);

int __ext4_journal_get_write_access(const char *where, unsigned int line,
				    handle_t *handle, struct buffer_head *bh);

int __ext4_forget(const char *where, unsigned int line, handle_t *handle,
		  int is_metadata, struct inode *inode,
		  struct buffer_head *bh, ext4_fsblk_t blocknr);

int __ext4_journal_get_create_access(const char *where, unsigned int line,
				handle_t *handle, struct buffer_head *bh);

int __ext4_handle_dirty_metadata(const char *where, unsigned int line,
				 handle_t *handle, struct inode *inode,
				 struct buffer_head *bh);

int __ext4_handle_dirty_super(const char *where, unsigned int line,
			      handle_t *handle, struct super_block *sb);

#define ext4_journal_get_write_access(handle, bh) \
	__ext4_journal_get_write_access(__func__, __LINE__, (handle), (bh))
#define ext4_forget(handle, is_metadata, inode, bh, block_nr) \
	__ext4_forget(__func__, __LINE__, (handle), (is_metadata), (inode), \
		      (bh), (block_nr))
#define ext4_journal_get_create_access(handle, bh) \
	__ext4_journal_get_create_access(__func__, __LINE__, (handle), (bh))
#define ext4_handle_dirty_metadata(handle, inode, bh) \
	__ext4_handle_dirty_metadata(__func__, __LINE__, (handle), (inode), \
				     (bh))
#define ext4_handle_dirty_super(handle, sb) \
	__ext4_handle_dirty_super(__func__, __LINE__, (handle), (sb))

handle_t *ext4_journal_start_sb(struct super_block *sb, int nblocks);
int __ext4_journal_stop(const char *where, unsigned int line, handle_t *handle);

#define EXT4_NOJOURNAL_MAX_REF_COUNT ((unsigned long) 4096)

static inline int ext4_handle_valid(handle_t *handle)
{
	if ((unsigned long)handle < EXT4_NOJOURNAL_MAX_REF_COUNT)
		return 0;
	return 1;
}

static inline void ext4_handle_sync(handle_t *handle)
{
	if (ext4_handle_valid(handle))
		handle->h_sync = 1;
}

static inline void ext4_handle_release_buffer(handle_t *handle,
						struct buffer_head *bh)
{
	if (ext4_handle_valid(handle))
		jbd2_journal_release_buffer(handle, bh);
}

static inline int ext4_handle_is_aborted(handle_t *handle)
{
	if (ext4_handle_valid(handle))
		return is_handle_aborted(handle);
	return 0;
}

static inline int ext4_handle_has_enough_credits(handle_t *handle, int needed)
{
	if (ext4_handle_valid(handle) && handle->h_buffer_credits < needed)
		return 0;
	return 1;
}

static inline handle_t *ext4_journal_start(struct inode *inode, int nblocks)
{
	return ext4_journal_start_sb(inode->i_sb, nblocks);
}

#define ext4_journal_stop(handle) \
	__ext4_journal_stop(__func__, __LINE__, (handle))

static inline handle_t *ext4_journal_current_handle(void)
{
	return journal_current_handle();
}

static inline int ext4_journal_extend(handle_t *handle, int nblocks)
{
	if (ext4_handle_valid(handle))
		return jbd2_journal_extend(handle, nblocks);
	return 0;
}

static inline int ext4_journal_restart(handle_t *handle, int nblocks)
{
	if (ext4_handle_valid(handle))
		return jbd2_journal_restart(handle, nblocks);
	return 0;
}

static inline int ext4_journal_blocks_per_page(struct inode *inode)
{
	if (EXT4_JOURNAL(inode) != NULL)
		return jbd2_journal_blocks_per_page(inode);
	return 0;
}

static inline int ext4_journal_force_commit(journal_t *journal)
{
	if (journal)
		return jbd2_journal_force_commit(journal);
	return 0;
}

static inline int ext4_jbd2_file_inode(handle_t *handle, struct inode *inode)
{
	if (ext4_handle_valid(handle))
		return jbd2_journal_file_inode(handle, EXT4_I(inode)->jinode);
	return 0;
}

static inline void ext4_update_inode_fsync_trans(handle_t *handle,
						 struct inode *inode,
						 int datasync)
{
	struct ext4_inode_info *ei = EXT4_I(inode);

	if (ext4_handle_valid(handle)) {
		ei->i_sync_tid = handle->h_transaction->t_tid;
		if (datasync)
			ei->i_datasync_tid = handle->h_transaction->t_tid;
	}
}

int ext4_force_commit(struct super_block *sb);

#define EXT4_INODE_JOURNAL_DATA_MODE	0x01 
#define EXT4_INODE_ORDERED_DATA_MODE	0x02 
#define EXT4_INODE_WRITEBACK_DATA_MODE	0x04 

static inline int ext4_inode_journal_mode(struct inode *inode)
{
	if (EXT4_JOURNAL(inode) == NULL)
		return EXT4_INODE_WRITEBACK_DATA_MODE;	
	
	if (!S_ISREG(inode->i_mode) ||
	    test_opt(inode->i_sb, DATA_FLAGS) == EXT4_MOUNT_JOURNAL_DATA)
		return EXT4_INODE_JOURNAL_DATA_MODE;	
	if (ext4_test_inode_flag(inode, EXT4_INODE_JOURNAL_DATA) &&
	    !test_opt(inode->i_sb, DELALLOC))
		return EXT4_INODE_JOURNAL_DATA_MODE;	
	if (test_opt(inode->i_sb, DATA_FLAGS) == EXT4_MOUNT_ORDERED_DATA)
		return EXT4_INODE_ORDERED_DATA_MODE;	
	if (test_opt(inode->i_sb, DATA_FLAGS) == EXT4_MOUNT_WRITEBACK_DATA)
		return EXT4_INODE_WRITEBACK_DATA_MODE;	
	else
		BUG();
}

static inline int ext4_should_journal_data(struct inode *inode)
{
	return ext4_inode_journal_mode(inode) & EXT4_INODE_JOURNAL_DATA_MODE;
}

static inline int ext4_should_order_data(struct inode *inode)
{
	return ext4_inode_journal_mode(inode) & EXT4_INODE_ORDERED_DATA_MODE;
}

static inline int ext4_should_writeback_data(struct inode *inode)
{
	return ext4_inode_journal_mode(inode) & EXT4_INODE_WRITEBACK_DATA_MODE;
}

static inline int ext4_should_dioread_nolock(struct inode *inode)
{
	if (!test_opt(inode->i_sb, DIOREAD_NOLOCK))
		return 0;
	if (!S_ISREG(inode->i_mode))
		return 0;
	if (!(ext4_test_inode_flag(inode, EXT4_INODE_EXTENTS)))
		return 0;
	if (ext4_should_journal_data(inode))
		return 0;
	return 1;
}

#endif	
