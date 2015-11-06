/*
 * Copyright (C) 2001 Jens Axboe <axboe@kernel.dk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public Licens
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-
 *
 */
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/mempool.h>
#include <linux/workqueue.h>
#include <scsi/sg.h>		

#include <trace/events/block.h>

#define BIO_INLINE_VECS		4

static mempool_t *bio_split_pool __read_mostly;

#define BV(x) { .nr_vecs = x, .name = "biovec-"__stringify(x) }
static struct biovec_slab bvec_slabs[BIOVEC_NR_POOLS] __read_mostly = {
	BV(1), BV(4), BV(16), BV(64), BV(128), BV(BIO_MAX_PAGES),
};
#undef BV

struct bio_set *fs_bio_set;

struct bio_slab {
	struct kmem_cache *slab;
	unsigned int slab_ref;
	unsigned int slab_size;
	char name[8];
};
static DEFINE_MUTEX(bio_slab_lock);
static struct bio_slab *bio_slabs;
static unsigned int bio_slab_nr, bio_slab_max;

static struct kmem_cache *bio_find_or_create_slab(unsigned int extra_size)
{
	unsigned int sz = sizeof(struct bio) + extra_size;
	struct kmem_cache *slab = NULL;
	struct bio_slab *bslab;
	unsigned int i, entry = -1;

	mutex_lock(&bio_slab_lock);

	i = 0;
	while (i < bio_slab_nr) {
		bslab = &bio_slabs[i];

		if (!bslab->slab && entry == -1)
			entry = i;
		else if (bslab->slab_size == sz) {
			slab = bslab->slab;
			bslab->slab_ref++;
			break;
		}
		i++;
	}

	if (slab)
		goto out_unlock;

	if (bio_slab_nr == bio_slab_max && entry == -1) {
		bio_slab_max <<= 1;
		bio_slabs = krealloc(bio_slabs,
				     bio_slab_max * sizeof(struct bio_slab),
				     GFP_KERNEL);
		if (!bio_slabs)
			goto out_unlock;
	}
	if (entry == -1)
		entry = bio_slab_nr++;

	bslab = &bio_slabs[entry];

	snprintf(bslab->name, sizeof(bslab->name), "bio-%d", entry);
	slab = kmem_cache_create(bslab->name, sz, 0, SLAB_HWCACHE_ALIGN, NULL);
	if (!slab)
		goto out_unlock;

	printk(KERN_INFO "bio: create slab <%s> at %d\n", bslab->name, entry);
	bslab->slab = slab;
	bslab->slab_ref = 1;
	bslab->slab_size = sz;
out_unlock:
	mutex_unlock(&bio_slab_lock);
	return slab;
}

static void bio_put_slab(struct bio_set *bs)
{
	struct bio_slab *bslab = NULL;
	unsigned int i;

	mutex_lock(&bio_slab_lock);

	for (i = 0; i < bio_slab_nr; i++) {
		if (bs->bio_slab == bio_slabs[i].slab) {
			bslab = &bio_slabs[i];
			break;
		}
	}

	if (WARN(!bslab, KERN_ERR "bio: unable to find slab!\n"))
		goto out;

	WARN_ON(!bslab->slab_ref);

	if (--bslab->slab_ref)
		goto out;

	kmem_cache_destroy(bslab->slab);
	bslab->slab = NULL;

out:
	mutex_unlock(&bio_slab_lock);
}

unsigned int bvec_nr_vecs(unsigned short idx)
{
	return bvec_slabs[idx].nr_vecs;
}

void bvec_free_bs(struct bio_set *bs, struct bio_vec *bv, unsigned int idx)
{
	BIO_BUG_ON(idx >= BIOVEC_NR_POOLS);

	if (idx == BIOVEC_MAX_IDX)
		mempool_free(bv, bs->bvec_pool);
	else {
		struct biovec_slab *bvs = bvec_slabs + idx;

		kmem_cache_free(bvs->slab, bv);
	}
}

struct bio_vec *bvec_alloc_bs(gfp_t gfp_mask, int nr, unsigned long *idx,
			      struct bio_set *bs)
{
	struct bio_vec *bvl;

	switch (nr) {
	case 1:
		*idx = 0;
		break;
	case 2 ... 4:
		*idx = 1;
		break;
	case 5 ... 16:
		*idx = 2;
		break;
	case 17 ... 64:
		*idx = 3;
		break;
	case 65 ... 128:
		*idx = 4;
		break;
	case 129 ... BIO_MAX_PAGES:
		*idx = 5;
		break;
	default:
		return NULL;
	}

	if (*idx == BIOVEC_MAX_IDX) {
fallback:
		bvl = mempool_alloc(bs->bvec_pool, gfp_mask);
	} else {
		struct biovec_slab *bvs = bvec_slabs + *idx;
		gfp_t __gfp_mask = gfp_mask & ~(__GFP_WAIT | __GFP_IO);

		__gfp_mask |= __GFP_NOMEMALLOC | __GFP_NORETRY | __GFP_NOWARN;

		bvl = kmem_cache_alloc(bvs->slab, __gfp_mask);
		if (unlikely(!bvl && (gfp_mask & __GFP_WAIT))) {
			*idx = BIOVEC_MAX_IDX;
			goto fallback;
		}
	}

	return bvl;
}

void bio_free(struct bio *bio, struct bio_set *bs)
{
	void *p;

	if (bio_has_allocated_vec(bio))
		bvec_free_bs(bs, bio->bi_io_vec, BIO_POOL_IDX(bio));

	if (bio_integrity(bio))
		bio_integrity_free(bio, bs);

	p = bio;
	if (bs->front_pad)
		p -= bs->front_pad;

	mempool_free(p, bs->bio_pool);
}
EXPORT_SYMBOL(bio_free);

void bio_init(struct bio *bio)
{
	memset(bio, 0, sizeof(*bio));
	bio->bi_flags = 1 << BIO_UPTODATE;
	atomic_set(&bio->bi_cnt, 1);
}
EXPORT_SYMBOL(bio_init);

struct bio *bio_alloc_bioset(gfp_t gfp_mask, int nr_iovecs, struct bio_set *bs)
{
	unsigned long idx = BIO_POOL_NONE;
	struct bio_vec *bvl = NULL;
	struct bio *bio;
	void *p;

	p = mempool_alloc(bs->bio_pool, gfp_mask);
	if (unlikely(!p))
		return NULL;
	bio = p + bs->front_pad;

	bio_init(bio);

	if (unlikely(!nr_iovecs))
		goto out_set;

	if (nr_iovecs <= BIO_INLINE_VECS) {
		bvl = bio->bi_inline_vecs;
		nr_iovecs = BIO_INLINE_VECS;
	} else {
		bvl = bvec_alloc_bs(gfp_mask, nr_iovecs, &idx, bs);
		if (unlikely(!bvl))
			goto err_free;

		nr_iovecs = bvec_nr_vecs(idx);
	}
out_set:
	bio->bi_flags |= idx << BIO_POOL_OFFSET;
	bio->bi_max_vecs = nr_iovecs;
	bio->bi_io_vec = bvl;
	return bio;

err_free:
	mempool_free(p, bs->bio_pool);
	return NULL;
}
EXPORT_SYMBOL(bio_alloc_bioset);

static void bio_fs_destructor(struct bio *bio)
{
	bio_free(bio, fs_bio_set);
}

struct bio *bio_alloc(gfp_t gfp_mask, unsigned int nr_iovecs)
{
	struct bio *bio = bio_alloc_bioset(gfp_mask, nr_iovecs, fs_bio_set);

	if (bio)
		bio->bi_destructor = bio_fs_destructor;

	return bio;
}
EXPORT_SYMBOL(bio_alloc);

static void bio_kmalloc_destructor(struct bio *bio)
{
	if (bio_integrity(bio))
		bio_integrity_free(bio, fs_bio_set);
	kfree(bio);
}

struct bio *bio_kmalloc(gfp_t gfp_mask, unsigned int nr_iovecs)
{
	struct bio *bio;

	if (nr_iovecs > UIO_MAXIOV)
		return NULL;

	bio = kmalloc(sizeof(struct bio) + nr_iovecs * sizeof(struct bio_vec),
		      gfp_mask);
	if (unlikely(!bio))
		return NULL;

	bio_init(bio);
	bio->bi_flags |= BIO_POOL_NONE << BIO_POOL_OFFSET;
	bio->bi_max_vecs = nr_iovecs;
	bio->bi_io_vec = bio->bi_inline_vecs;
	bio->bi_destructor = bio_kmalloc_destructor;

	return bio;
}
EXPORT_SYMBOL(bio_kmalloc);

void zero_fill_bio(struct bio *bio)
{
	unsigned long flags;
	struct bio_vec *bv;
	int i;

	bio_for_each_segment(bv, bio, i) {
		char *data = bvec_kmap_irq(bv, &flags);
		memset(data, 0, bv->bv_len);
		flush_dcache_page(bv->bv_page);
		bvec_kunmap_irq(data, &flags);
	}
}
EXPORT_SYMBOL(zero_fill_bio);

void bio_put(struct bio *bio)
{
	BIO_BUG_ON(!atomic_read(&bio->bi_cnt));

	if (atomic_dec_and_test(&bio->bi_cnt)) {
		bio->bi_next = NULL;
		bio->bi_destructor(bio);
	}
}
EXPORT_SYMBOL(bio_put);

inline int bio_phys_segments(struct request_queue *q, struct bio *bio)
{
	if (unlikely(!bio_flagged(bio, BIO_SEG_VALID)))
		blk_recount_segments(q, bio);

	return bio->bi_phys_segments;
}
EXPORT_SYMBOL(bio_phys_segments);

void __bio_clone(struct bio *bio, struct bio *bio_src)
{
	memcpy(bio->bi_io_vec, bio_src->bi_io_vec,
		bio_src->bi_max_vecs * sizeof(struct bio_vec));

	bio->bi_sector = bio_src->bi_sector;
	bio->bi_bdev = bio_src->bi_bdev;
	bio->bi_flags |= 1 << BIO_CLONED;
	bio->bi_rw = bio_src->bi_rw;
	bio->bi_vcnt = bio_src->bi_vcnt;
	bio->bi_size = bio_src->bi_size;
	bio->bi_idx = bio_src->bi_idx;
}
EXPORT_SYMBOL(__bio_clone);

struct bio *bio_clone(struct bio *bio, gfp_t gfp_mask)
{
	struct bio *b = bio_alloc_bioset(gfp_mask, bio->bi_max_vecs, fs_bio_set);

	if (!b)
		return NULL;

	b->bi_destructor = bio_fs_destructor;
	__bio_clone(b, bio);

	if (bio_integrity(bio)) {
		int ret;

		ret = bio_integrity_clone(b, bio, gfp_mask, fs_bio_set);

		if (ret < 0) {
			bio_put(b);
			return NULL;
		}
	}

	return b;
}
EXPORT_SYMBOL(bio_clone);

int bio_get_nr_vecs(struct block_device *bdev)
{
	struct request_queue *q = bdev_get_queue(bdev);
	int nr_pages;

	nr_pages = min_t(unsigned,
		     queue_max_segments(q),
		     queue_max_sectors(q) / (PAGE_SIZE >> 9) + 1);

	return min_t(unsigned, nr_pages, BIO_MAX_PAGES);

}
EXPORT_SYMBOL(bio_get_nr_vecs);

static int __bio_add_page(struct request_queue *q, struct bio *bio, struct page
			  *page, unsigned int len, unsigned int offset,
			  unsigned short max_sectors)
{
	int retried_segments = 0;
	struct bio_vec *bvec;

	if (unlikely(bio_flagged(bio, BIO_CLONED)))
		return 0;

	if (((bio->bi_size + len) >> 9) > max_sectors)
		return 0;

	if (bio->bi_vcnt > 0) {
		struct bio_vec *prev = &bio->bi_io_vec[bio->bi_vcnt - 1];

		if (page == prev->bv_page &&
		    offset == prev->bv_offset + prev->bv_len) {
			unsigned int prev_bv_len = prev->bv_len;
			prev->bv_len += len;

			if (q->merge_bvec_fn) {
				struct bvec_merge_data bvm = {
					.bi_bdev = bio->bi_bdev,
					.bi_sector = bio->bi_sector,
					.bi_size = bio->bi_size - prev_bv_len,
					.bi_rw = bio->bi_rw,
				};

				if (q->merge_bvec_fn(q, &bvm, prev) < prev->bv_len) {
					prev->bv_len -= len;
					return 0;
				}
			}

			goto done;
		}
	}

	if (bio->bi_vcnt >= bio->bi_max_vecs)
		return 0;


	while (bio->bi_phys_segments >= queue_max_segments(q)) {

		if (retried_segments)
			return 0;

		retried_segments = 1;
		blk_recount_segments(q, bio);
	}

	bvec = &bio->bi_io_vec[bio->bi_vcnt];
	bvec->bv_page = page;
	bvec->bv_len = len;
	bvec->bv_offset = offset;

	if (q->merge_bvec_fn) {
		struct bvec_merge_data bvm = {
			.bi_bdev = bio->bi_bdev,
			.bi_sector = bio->bi_sector,
			.bi_size = bio->bi_size,
			.bi_rw = bio->bi_rw,
		};

		if (q->merge_bvec_fn(q, &bvm, bvec) < bvec->bv_len) {
			bvec->bv_page = NULL;
			bvec->bv_len = 0;
			bvec->bv_offset = 0;
			return 0;
		}
	}

	
	if (bio->bi_vcnt && (BIOVEC_PHYS_MERGEABLE(bvec-1, bvec)))
		bio->bi_flags &= ~(1 << BIO_SEG_VALID);

	bio->bi_vcnt++;
	bio->bi_phys_segments++;
 done:
	bio->bi_size += len;
	return len;
}

int bio_add_pc_page(struct request_queue *q, struct bio *bio, struct page *page,
		    unsigned int len, unsigned int offset)
{
	return __bio_add_page(q, bio, page, len, offset,
			      queue_max_hw_sectors(q));
}
EXPORT_SYMBOL(bio_add_pc_page);

int bio_add_page(struct bio *bio, struct page *page, unsigned int len,
		 unsigned int offset)
{
	struct request_queue *q = bdev_get_queue(bio->bi_bdev);
	return __bio_add_page(q, bio, page, len, offset, queue_max_sectors(q));
}
EXPORT_SYMBOL(bio_add_page);

struct bio_map_data {
	struct bio_vec *iovecs;
	struct sg_iovec *sgvecs;
	int nr_sgvecs;
	int is_our_pages;
};

static void bio_set_map_data(struct bio_map_data *bmd, struct bio *bio,
			     struct sg_iovec *iov, int iov_count,
			     int is_our_pages)
{
	memcpy(bmd->iovecs, bio->bi_io_vec, sizeof(struct bio_vec) * bio->bi_vcnt);
	memcpy(bmd->sgvecs, iov, sizeof(struct sg_iovec) * iov_count);
	bmd->nr_sgvecs = iov_count;
	bmd->is_our_pages = is_our_pages;
	bio->bi_private = bmd;
}

static void bio_free_map_data(struct bio_map_data *bmd)
{
	kfree(bmd->iovecs);
	kfree(bmd->sgvecs);
	kfree(bmd);
}

static struct bio_map_data *bio_alloc_map_data(int nr_segs,
					       unsigned int iov_count,
					       gfp_t gfp_mask)
{
	struct bio_map_data *bmd;

	if (iov_count > UIO_MAXIOV)
		return NULL;

	bmd = kmalloc(sizeof(*bmd), gfp_mask);
	if (!bmd)
		return NULL;

	bmd->iovecs = kmalloc(sizeof(struct bio_vec) * nr_segs, gfp_mask);
	if (!bmd->iovecs) {
		kfree(bmd);
		return NULL;
	}

	bmd->sgvecs = kmalloc(sizeof(struct sg_iovec) * iov_count, gfp_mask);
	if (bmd->sgvecs)
		return bmd;

	kfree(bmd->iovecs);
	kfree(bmd);
	return NULL;
}

static int __bio_copy_iov(struct bio *bio, struct bio_vec *iovecs,
			  struct sg_iovec *iov, int iov_count,
			  int to_user, int from_user, int do_free_page)
{
	int ret = 0, i;
	struct bio_vec *bvec;
	int iov_idx = 0;
	unsigned int iov_off = 0;

	__bio_for_each_segment(bvec, bio, i, 0) {
		char *bv_addr = page_address(bvec->bv_page);
		unsigned int bv_len = iovecs[i].bv_len;

		while (bv_len && iov_idx < iov_count) {
			unsigned int bytes;
			char __user *iov_addr;

			bytes = min_t(unsigned int,
				      iov[iov_idx].iov_len - iov_off, bv_len);
			iov_addr = iov[iov_idx].iov_base + iov_off;

			if (!ret) {
				if (to_user)
					ret = copy_to_user(iov_addr, bv_addr,
							   bytes);

				if (from_user)
					ret = copy_from_user(bv_addr, iov_addr,
							     bytes);

				if (ret)
					ret = -EFAULT;
			}

			bv_len -= bytes;
			bv_addr += bytes;
			iov_addr += bytes;
			iov_off += bytes;

			if (iov[iov_idx].iov_len == iov_off) {
				iov_idx++;
				iov_off = 0;
			}
		}

		if (do_free_page)
			__free_page(bvec->bv_page);
	}

	return ret;
}

int bio_uncopy_user(struct bio *bio)
{
	struct bio_map_data *bmd = bio->bi_private;
	int ret = 0;

	if (!bio_flagged(bio, BIO_NULL_MAPPED))
		ret = __bio_copy_iov(bio, bmd->iovecs, bmd->sgvecs,
				     bmd->nr_sgvecs, bio_data_dir(bio) == READ,
				     0, bmd->is_our_pages);
	bio_free_map_data(bmd);
	bio_put(bio);
	return ret;
}
EXPORT_SYMBOL(bio_uncopy_user);

struct bio *bio_copy_user_iov(struct request_queue *q,
			      struct rq_map_data *map_data,
			      struct sg_iovec *iov, int iov_count,
			      int write_to_vm, gfp_t gfp_mask)
{
	struct bio_map_data *bmd;
	struct bio_vec *bvec;
	struct page *page;
	struct bio *bio;
	int i, ret;
	int nr_pages = 0;
	unsigned int len = 0;
	unsigned int offset = map_data ? map_data->offset & ~PAGE_MASK : 0;

	for (i = 0; i < iov_count; i++) {
		unsigned long uaddr;
		unsigned long end;
		unsigned long start;

		uaddr = (unsigned long)iov[i].iov_base;
		end = (uaddr + iov[i].iov_len + PAGE_SIZE - 1) >> PAGE_SHIFT;
		start = uaddr >> PAGE_SHIFT;

		if (end < start)
			return ERR_PTR(-EINVAL);

		nr_pages += end - start;
		len += iov[i].iov_len;
	}

	if (offset)
		nr_pages++;

	bmd = bio_alloc_map_data(nr_pages, iov_count, gfp_mask);
	if (!bmd)
		return ERR_PTR(-ENOMEM);

	ret = -ENOMEM;
	bio = bio_kmalloc(gfp_mask, nr_pages);
	if (!bio)
		goto out_bmd;

	if (!write_to_vm)
		bio->bi_rw |= REQ_WRITE;

	ret = 0;

	if (map_data) {
		nr_pages = 1 << map_data->page_order;
		i = map_data->offset / PAGE_SIZE;
	}
	while (len) {
		unsigned int bytes = PAGE_SIZE;

		bytes -= offset;

		if (bytes > len)
			bytes = len;

		if (map_data) {
			if (i == map_data->nr_entries * nr_pages) {
				ret = -ENOMEM;
				break;
			}

			page = map_data->pages[i / nr_pages];
			page += (i % nr_pages);

			i++;
		} else {
			page = alloc_page(q->bounce_gfp | gfp_mask);
			if (!page) {
				ret = -ENOMEM;
				break;
			}
		}

		if (bio_add_pc_page(q, bio, page, bytes, offset) < bytes)
			break;

		len -= bytes;
		offset = 0;
	}

	if (ret)
		goto cleanup;

	if ((!write_to_vm && (!map_data || !map_data->null_mapped)) ||
	    (map_data && map_data->from_user)) {
		ret = __bio_copy_iov(bio, bio->bi_io_vec, iov, iov_count, 0, 1, 0);
		if (ret)
			goto cleanup;
	}

	bio_set_map_data(bmd, bio, iov, iov_count, map_data ? 0 : 1);
	return bio;
cleanup:
	if (!map_data)
		bio_for_each_segment(bvec, bio, i)
			__free_page(bvec->bv_page);

	bio_put(bio);
out_bmd:
	bio_free_map_data(bmd);
	return ERR_PTR(ret);
}

struct bio *bio_copy_user(struct request_queue *q, struct rq_map_data *map_data,
			  unsigned long uaddr, unsigned int len,
			  int write_to_vm, gfp_t gfp_mask)
{
	struct sg_iovec iov;

	iov.iov_base = (void __user *)uaddr;
	iov.iov_len = len;

	return bio_copy_user_iov(q, map_data, &iov, 1, write_to_vm, gfp_mask);
}
EXPORT_SYMBOL(bio_copy_user);

static struct bio *__bio_map_user_iov(struct request_queue *q,
				      struct block_device *bdev,
				      struct sg_iovec *iov, int iov_count,
				      int write_to_vm, gfp_t gfp_mask)
{
	int i, j;
	int nr_pages = 0;
	struct page **pages;
	struct bio *bio;
	int cur_page = 0;
	int ret, offset;

	for (i = 0; i < iov_count; i++) {
		unsigned long uaddr = (unsigned long)iov[i].iov_base;
		unsigned long len = iov[i].iov_len;
		unsigned long end = (uaddr + len + PAGE_SIZE - 1) >> PAGE_SHIFT;
		unsigned long start = uaddr >> PAGE_SHIFT;

		if (end < start)
			return ERR_PTR(-EINVAL);

		nr_pages += end - start;
		if (uaddr & queue_dma_alignment(q))
			return ERR_PTR(-EINVAL);
	}

	if (!nr_pages)
		return ERR_PTR(-EINVAL);

	bio = bio_kmalloc(gfp_mask, nr_pages);
	if (!bio)
		return ERR_PTR(-ENOMEM);

	ret = -ENOMEM;
	pages = kcalloc(nr_pages, sizeof(struct page *), gfp_mask);
	if (!pages)
		goto out;

	for (i = 0; i < iov_count; i++) {
		unsigned long uaddr = (unsigned long)iov[i].iov_base;
		unsigned long len = iov[i].iov_len;
		unsigned long end = (uaddr + len + PAGE_SIZE - 1) >> PAGE_SHIFT;
		unsigned long start = uaddr >> PAGE_SHIFT;
		const int local_nr_pages = end - start;
		const int page_limit = cur_page + local_nr_pages;

		ret = get_user_pages_fast(uaddr, local_nr_pages,
				write_to_vm, &pages[cur_page]);
		if (ret < local_nr_pages) {
			ret = -EFAULT;
			goto out_unmap;
		}

		offset = uaddr & ~PAGE_MASK;
		for (j = cur_page; j < page_limit; j++) {
			unsigned int bytes = PAGE_SIZE - offset;

			if (len <= 0)
				break;
			
			if (bytes > len)
				bytes = len;

			if (bio_add_pc_page(q, bio, pages[j], bytes, offset) <
					    bytes)
				break;

			len -= bytes;
			offset = 0;
		}

		cur_page = j;
		while (j < page_limit)
			page_cache_release(pages[j++]);
	}

	kfree(pages);

	if (!write_to_vm)
		bio->bi_rw |= REQ_WRITE;

	bio->bi_bdev = bdev;
	bio->bi_flags |= (1 << BIO_USER_MAPPED);
	return bio;

 out_unmap:
	for (i = 0; i < nr_pages; i++) {
		if(!pages[i])
			break;
		page_cache_release(pages[i]);
	}
 out:
	kfree(pages);
	bio_put(bio);
	return ERR_PTR(ret);
}

struct bio *bio_map_user(struct request_queue *q, struct block_device *bdev,
			 unsigned long uaddr, unsigned int len, int write_to_vm,
			 gfp_t gfp_mask)
{
	struct sg_iovec iov;

	iov.iov_base = (void __user *)uaddr;
	iov.iov_len = len;

	return bio_map_user_iov(q, bdev, &iov, 1, write_to_vm, gfp_mask);
}
EXPORT_SYMBOL(bio_map_user);

struct bio *bio_map_user_iov(struct request_queue *q, struct block_device *bdev,
			     struct sg_iovec *iov, int iov_count,
			     int write_to_vm, gfp_t gfp_mask)
{
	struct bio *bio;

	bio = __bio_map_user_iov(q, bdev, iov, iov_count, write_to_vm,
				 gfp_mask);
	if (IS_ERR(bio))
		return bio;

	bio_get(bio);

	return bio;
}

static void __bio_unmap_user(struct bio *bio)
{
	struct bio_vec *bvec;
	int i;

	__bio_for_each_segment(bvec, bio, i, 0) {
		if (bio_data_dir(bio) == READ)
			set_page_dirty_lock(bvec->bv_page);

		page_cache_release(bvec->bv_page);
	}

	bio_put(bio);
}

void bio_unmap_user(struct bio *bio)
{
	__bio_unmap_user(bio);
	bio_put(bio);
}
EXPORT_SYMBOL(bio_unmap_user);

static void bio_map_kern_endio(struct bio *bio, int err)
{
	bio_put(bio);
}

static struct bio *__bio_map_kern(struct request_queue *q, void *data,
				  unsigned int len, gfp_t gfp_mask)
{
	unsigned long kaddr = (unsigned long)data;
	unsigned long end = (kaddr + len + PAGE_SIZE - 1) >> PAGE_SHIFT;
	unsigned long start = kaddr >> PAGE_SHIFT;
	const int nr_pages = end - start;
	int offset, i;
	struct bio *bio;

	bio = bio_kmalloc(gfp_mask, nr_pages);
	if (!bio)
		return ERR_PTR(-ENOMEM);

	offset = offset_in_page(kaddr);
	for (i = 0; i < nr_pages; i++) {
		unsigned int bytes = PAGE_SIZE - offset;

		if (len <= 0)
			break;

		if (bytes > len)
			bytes = len;

		if (bio_add_pc_page(q, bio, virt_to_page(data), bytes,
				    offset) < bytes)
			break;

		data += bytes;
		len -= bytes;
		offset = 0;
	}

	bio->bi_end_io = bio_map_kern_endio;
	return bio;
}

struct bio *bio_map_kern(struct request_queue *q, void *data, unsigned int len,
			 gfp_t gfp_mask)
{
	struct bio *bio;

	bio = __bio_map_kern(q, data, len, gfp_mask);
	if (IS_ERR(bio))
		return bio;

	if (bio->bi_size == len)
		return bio;

	bio_put(bio);
	return ERR_PTR(-EINVAL);
}
EXPORT_SYMBOL(bio_map_kern);

static void bio_copy_kern_endio(struct bio *bio, int err)
{
	struct bio_vec *bvec;
	const int read = bio_data_dir(bio) == READ;
	struct bio_map_data *bmd = bio->bi_private;
	int i;
	char *p = bmd->sgvecs[0].iov_base;

	__bio_for_each_segment(bvec, bio, i, 0) {
		char *addr = page_address(bvec->bv_page);
		int len = bmd->iovecs[i].bv_len;

		if (read)
			memcpy(p, addr, len);

		__free_page(bvec->bv_page);
		p += len;
	}

	bio_free_map_data(bmd);
	bio_put(bio);
}

struct bio *bio_copy_kern(struct request_queue *q, void *data, unsigned int len,
			  gfp_t gfp_mask, int reading)
{
	struct bio *bio;
	struct bio_vec *bvec;
	int i;

	bio = bio_copy_user(q, NULL, (unsigned long)data, len, 1, gfp_mask);
	if (IS_ERR(bio))
		return bio;

	if (!reading) {
		void *p = data;

		bio_for_each_segment(bvec, bio, i) {
			char *addr = page_address(bvec->bv_page);

			memcpy(addr, p, bvec->bv_len);
			p += bvec->bv_len;
		}
	}

	bio->bi_end_io = bio_copy_kern_endio;

	return bio;
}
EXPORT_SYMBOL(bio_copy_kern);


void bio_set_pages_dirty(struct bio *bio)
{
	struct bio_vec *bvec = bio->bi_io_vec;
	int i;

	for (i = 0; i < bio->bi_vcnt; i++) {
		struct page *page = bvec[i].bv_page;

		if (page && !PageCompound(page))
			set_page_dirty_lock(page);
	}
}

static void bio_release_pages(struct bio *bio)
{
	struct bio_vec *bvec = bio->bi_io_vec;
	int i;

	for (i = 0; i < bio->bi_vcnt; i++) {
		struct page *page = bvec[i].bv_page;

		if (page)
			put_page(page);
	}
}


static void bio_dirty_fn(struct work_struct *work);

static DECLARE_WORK(bio_dirty_work, bio_dirty_fn);
static DEFINE_SPINLOCK(bio_dirty_lock);
static struct bio *bio_dirty_list;

static void bio_dirty_fn(struct work_struct *work)
{
	unsigned long flags;
	struct bio *bio;

	spin_lock_irqsave(&bio_dirty_lock, flags);
	bio = bio_dirty_list;
	bio_dirty_list = NULL;
	spin_unlock_irqrestore(&bio_dirty_lock, flags);

	while (bio) {
		struct bio *next = bio->bi_private;

		bio_set_pages_dirty(bio);
		bio_release_pages(bio);
		bio_put(bio);
		bio = next;
	}
}

void bio_check_pages_dirty(struct bio *bio)
{
	struct bio_vec *bvec = bio->bi_io_vec;
	int nr_clean_pages = 0;
	int i;

	for (i = 0; i < bio->bi_vcnt; i++) {
		struct page *page = bvec[i].bv_page;

		if (PageDirty(page) || PageCompound(page)) {
			page_cache_release(page);
			bvec[i].bv_page = NULL;
		} else {
			nr_clean_pages++;
		}
	}

	if (nr_clean_pages) {
		unsigned long flags;

		spin_lock_irqsave(&bio_dirty_lock, flags);
		bio->bi_private = bio_dirty_list;
		bio_dirty_list = bio;
		spin_unlock_irqrestore(&bio_dirty_lock, flags);
		schedule_work(&bio_dirty_work);
	} else {
		bio_put(bio);
	}
}

#if ARCH_IMPLEMENTS_FLUSH_DCACHE_PAGE
void bio_flush_dcache_pages(struct bio *bi)
{
	int i;
	struct bio_vec *bvec;

	bio_for_each_segment(bvec, bi, i)
		flush_dcache_page(bvec->bv_page);
}
EXPORT_SYMBOL(bio_flush_dcache_pages);
#endif

void bio_endio(struct bio *bio, int error)
{
	if (error)
		clear_bit(BIO_UPTODATE, &bio->bi_flags);
	else if (!test_bit(BIO_UPTODATE, &bio->bi_flags))
		error = -EIO;

	if (bio->bi_end_io)
		bio->bi_end_io(bio, error);
}
EXPORT_SYMBOL(bio_endio);

void bio_pair_release(struct bio_pair *bp)
{
	if (atomic_dec_and_test(&bp->cnt)) {
		struct bio *master = bp->bio1.bi_private;

		bio_endio(master, bp->error);
		mempool_free(bp, bp->bio2.bi_private);
	}
}
EXPORT_SYMBOL(bio_pair_release);

static void bio_pair_end_1(struct bio *bi, int err)
{
	struct bio_pair *bp = container_of(bi, struct bio_pair, bio1);

	if (err)
		bp->error = err;

	bio_pair_release(bp);
}

static void bio_pair_end_2(struct bio *bi, int err)
{
	struct bio_pair *bp = container_of(bi, struct bio_pair, bio2);

	if (err)
		bp->error = err;

	bio_pair_release(bp);
}

struct bio_pair *bio_split(struct bio *bi, int first_sectors)
{
	struct bio_pair *bp = mempool_alloc(bio_split_pool, GFP_NOIO);

	if (!bp)
		return bp;

	trace_block_split(bdev_get_queue(bi->bi_bdev), bi,
				bi->bi_sector + first_sectors);

	BUG_ON(bi->bi_vcnt != 1);
	BUG_ON(bi->bi_idx != 0);
	atomic_set(&bp->cnt, 3);
	bp->error = 0;
	bp->bio1 = *bi;
	bp->bio2 = *bi;
	bp->bio2.bi_sector += first_sectors;
	bp->bio2.bi_size -= first_sectors << 9;
	bp->bio1.bi_size = first_sectors << 9;

	bp->bv1 = bi->bi_io_vec[0];
	bp->bv2 = bi->bi_io_vec[0];
	bp->bv2.bv_offset += first_sectors << 9;
	bp->bv2.bv_len -= first_sectors << 9;
	bp->bv1.bv_len = first_sectors << 9;

	bp->bio1.bi_io_vec = &bp->bv1;
	bp->bio2.bi_io_vec = &bp->bv2;

	bp->bio1.bi_max_vecs = 1;
	bp->bio2.bi_max_vecs = 1;

	bp->bio1.bi_end_io = bio_pair_end_1;
	bp->bio2.bi_end_io = bio_pair_end_2;

	bp->bio1.bi_private = bi;
	bp->bio2.bi_private = bio_split_pool;

	if (bio_integrity(bi))
		bio_integrity_split(bi, bp, first_sectors);

	return bp;
}
EXPORT_SYMBOL(bio_split);

sector_t bio_sector_offset(struct bio *bio, unsigned short index,
			   unsigned int offset)
{
	unsigned int sector_sz;
	struct bio_vec *bv;
	sector_t sectors;
	int i;

	sector_sz = queue_logical_block_size(bio->bi_bdev->bd_disk->queue);
	sectors = 0;

	if (index >= bio->bi_idx)
		index = bio->bi_vcnt - 1;

	__bio_for_each_segment(bv, bio, i, 0) {
		if (i == index) {
			if (offset > bv->bv_offset)
				sectors += (offset - bv->bv_offset) / sector_sz;
			break;
		}

		sectors += bv->bv_len / sector_sz;
	}

	return sectors;
}
EXPORT_SYMBOL(bio_sector_offset);

static int biovec_create_pools(struct bio_set *bs, int pool_entries)
{
	struct biovec_slab *bp = bvec_slabs + BIOVEC_MAX_IDX;

	bs->bvec_pool = mempool_create_slab_pool(pool_entries, bp->slab);
	if (!bs->bvec_pool)
		return -ENOMEM;

	return 0;
}

static void biovec_free_pools(struct bio_set *bs)
{
	mempool_destroy(bs->bvec_pool);
}

void bioset_free(struct bio_set *bs)
{
	if (bs->bio_pool)
		mempool_destroy(bs->bio_pool);

	bioset_integrity_free(bs);
	biovec_free_pools(bs);
	bio_put_slab(bs);

	kfree(bs);
}
EXPORT_SYMBOL(bioset_free);

struct bio_set *bioset_create(unsigned int pool_size, unsigned int front_pad)
{
	unsigned int back_pad = BIO_INLINE_VECS * sizeof(struct bio_vec);
	struct bio_set *bs;

	bs = kzalloc(sizeof(*bs), GFP_KERNEL);
	if (!bs)
		return NULL;

	bs->front_pad = front_pad;

	bs->bio_slab = bio_find_or_create_slab(front_pad + back_pad);
	if (!bs->bio_slab) {
		kfree(bs);
		return NULL;
	}

	bs->bio_pool = mempool_create_slab_pool(pool_size, bs->bio_slab);
	if (!bs->bio_pool)
		goto bad;

	if (!biovec_create_pools(bs, pool_size))
		return bs;

bad:
	bioset_free(bs);
	return NULL;
}
EXPORT_SYMBOL(bioset_create);

static void __init biovec_init_slabs(void)
{
	int i;

	for (i = 0; i < BIOVEC_NR_POOLS; i++) {
		int size;
		struct biovec_slab *bvs = bvec_slabs + i;

		if (bvs->nr_vecs <= BIO_INLINE_VECS) {
			bvs->slab = NULL;
			continue;
		}

		size = bvs->nr_vecs * sizeof(struct bio_vec);
		bvs->slab = kmem_cache_create(bvs->name, size, 0,
                                SLAB_HWCACHE_ALIGN|SLAB_PANIC, NULL);
	}
}

static int __init init_bio(void)
{
	bio_slab_max = 2;
	bio_slab_nr = 0;
	bio_slabs = kzalloc(bio_slab_max * sizeof(struct bio_slab), GFP_KERNEL);
	if (!bio_slabs)
		panic("bio: can't allocate bios\n");

	bio_integrity_init();
	biovec_init_slabs();

	fs_bio_set = bioset_create(BIO_POOL_SIZE, 0);
	if (!fs_bio_set)
		panic("bio: can't allocate bios\n");

	if (bioset_integrity_create(fs_bio_set, BIO_POOL_SIZE))
		panic("bio: can't create integrity pool\n");

	bio_split_pool = mempool_create_kmalloc_pool(BIO_SPLIT_ENTRIES,
						     sizeof(struct bio_pair));
	if (!bio_split_pool)
		panic("bio: can't create split pool\n");

	return 0;
}
subsys_initcall(init_bio);
