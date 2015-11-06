#ifndef WRITEBACK_H
#define WRITEBACK_H

#include <linux/sched.h>
#include <linux/fs.h>

DECLARE_PER_CPU(int, dirty_throttle_leaks);

#define DIRTY_SCOPE		8
#define DIRTY_FULL_SCOPE	(DIRTY_SCOPE / 2)

struct backing_dev_info;

enum writeback_sync_modes {
	WB_SYNC_NONE,	
	WB_SYNC_ALL,	
};

enum wb_reason {
	WB_REASON_BACKGROUND,
	WB_REASON_TRY_TO_FREE_PAGES,
	WB_REASON_SYNC,
	WB_REASON_PERIODIC,
	WB_REASON_LAPTOP_TIMER,
	WB_REASON_FREE_MORE_MEM,
	WB_REASON_FS_FREE_SPACE,
	WB_REASON_FORKER_THREAD,

	WB_REASON_MAX,
};
extern const char *wb_reason_name[];

struct writeback_control {
	enum writeback_sync_modes sync_mode;
	long nr_to_write;		
	long pages_skipped;		

	loff_t range_start;
	loff_t range_end;

	unsigned for_kupdate:1;		
	unsigned for_background:1;	
	unsigned tagged_writepages:1;	
	unsigned for_reclaim:1;		
	unsigned range_cyclic:1;	
};

	
struct bdi_writeback;
int inode_wait(void *);
void writeback_inodes_sb(struct super_block *, enum wb_reason reason);
void writeback_inodes_sb_nr(struct super_block *, unsigned long nr,
							enum wb_reason reason);
int writeback_inodes_sb_if_idle(struct super_block *, enum wb_reason reason);
int writeback_inodes_sb_nr_if_idle(struct super_block *, unsigned long nr,
							enum wb_reason reason);
void sync_inodes_sb(struct super_block *);
long writeback_inodes_wb(struct bdi_writeback *wb, long nr_pages,
				enum wb_reason reason);
long wb_do_writeback(struct bdi_writeback *wb, int force_wait);
void wakeup_flusher_threads(long nr_pages, enum wb_reason reason);

static inline void wait_on_inode(struct inode *inode)
{
	might_sleep();
	wait_on_bit(&inode->i_state, __I_NEW, inode_wait, TASK_UNINTERRUPTIBLE);
}
static inline void inode_sync_wait(struct inode *inode)
{
	might_sleep();
	wait_on_bit(&inode->i_state, __I_SYNC, inode_wait,
							TASK_UNINTERRUPTIBLE);
}


#ifdef CONFIG_BLOCK
void laptop_io_completion(struct backing_dev_info *info);
void laptop_sync_completion(void);
void laptop_mode_sync(struct work_struct *work);
void laptop_mode_timer_fn(unsigned long data);
#else
static inline void laptop_sync_completion(void) { }
#endif
void throttle_vm_writeout(gfp_t gfp_mask);
bool zone_dirty_ok(struct zone *zone);

extern unsigned long global_dirty_limit;

extern int dirty_background_ratio;
extern unsigned long dirty_background_bytes;
extern int vm_dirty_ratio;
extern unsigned long vm_dirty_bytes;
extern unsigned int dirty_writeback_interval;
extern unsigned int dirty_expire_interval;
extern int vm_highmem_is_dirtyable;
extern int block_dump;
extern int laptop_mode;

extern int dirty_background_ratio_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos);
extern int dirty_background_bytes_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos);
extern int dirty_ratio_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos);
extern int dirty_bytes_handler(struct ctl_table *table, int write,
		void __user *buffer, size_t *lenp,
		loff_t *ppos);

struct ctl_table;
int dirty_writeback_centisecs_handler(struct ctl_table *, int,
				      void __user *, size_t *, loff_t *);

void global_dirty_limits(unsigned long *pbackground, unsigned long *pdirty);
unsigned long bdi_dirty_limit(struct backing_dev_info *bdi,
			       unsigned long dirty);

void __bdi_update_bandwidth(struct backing_dev_info *bdi,
			    unsigned long thresh,
			    unsigned long bg_thresh,
			    unsigned long dirty,
			    unsigned long bdi_thresh,
			    unsigned long bdi_dirty,
			    unsigned long start_time);

void page_writeback_init(void);
void balance_dirty_pages_ratelimited_nr(struct address_space *mapping,
					unsigned long nr_pages_dirtied);

static inline void
balance_dirty_pages_ratelimited(struct address_space *mapping)
{
	balance_dirty_pages_ratelimited_nr(mapping, 1);
}

typedef int (*writepage_t)(struct page *page, struct writeback_control *wbc,
				void *data);

int generic_writepages(struct address_space *mapping,
		       struct writeback_control *wbc);
void tag_pages_for_writeback(struct address_space *mapping,
			     pgoff_t start, pgoff_t end);
int write_cache_pages(struct address_space *mapping,
		      struct writeback_control *wbc, writepage_t writepage,
		      void *data);
int do_writepages(struct address_space *mapping, struct writeback_control *wbc);
void set_page_dirty_balance(struct page *page, int page_mkwrite);
void writeback_set_ratelimit(void);
void tag_pages_for_writeback(struct address_space *mapping,
			     pgoff_t start, pgoff_t end);

void account_page_redirty(struct page *page);

extern int nr_pdflush_threads;	


#endif		
