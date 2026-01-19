// SPDX-License-Identifier: GPL-2.0
/*
 * Workingset detection
 *
 * Copyright (C) 2013 Red Hat, Inc., Johannes Weiner
 */

#include <linux/memcontrol.h>
#include <linux/mm_inline.h>
#include <linux/writeback.h>
#include <linux/shmem_fs.h>
#include <linux/pagemap.h>
#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/swap.h>
#include <linux/dax.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include "internal.h"

/*
 *		Double CLOCK lists
 *
 * Per node, two clock lists are maintained for file pages: the
 * inactive and the active list.  Freshly faulted pages start out at
 * the head of the inactive list and page reclaim scans pages from the
 * tail.  Pages that are accessed multiple times on the inactive list
 * are promoted to the active list, to protect them from reclaim,
 * whereas(然而) active pages are demoted to the inactive list when the
 * active list grows too big.
 *
 *   fault ------------------------+
 *                                 |
 *              +--------------+   |            +-------------+
 *   reclaim <- |   inactive   | <-+-- demotion |    active   | <--+
 *              +--------------+                +-------------+    |
 *                     |                                           |
 *                     +-------------- promotion ------------------+
 *
 *
 *		Access frequency and refault distance
 *
 * A workload is thrashing when its pages are frequently used but they
 * are evicted from the inactive list every time before another access
 * would have promoted them to the active list.
 *
 * In cases where the average access distance between thrashing pages
 * is bigger than the size of memory there is nothing that can be
 * done - the thrashing set could never fit into memory under any
 * circumstance.
 *
 * However, the average access distance could be bigger than the
 * inactive list, yet smaller than the size of memory.  In this case,
 * the set could fit into memory if it weren't for the currently
 * active pages - which may be used more, hopefully less frequently:
 *
 *      +-memory available to cache-+
 *      |                           |
 *      +-inactive------+-active----+
 *  a b | c d e f g h i | J K L M N |
 *      +---------------+-----------+
 *
 * It is prohibitively expensive to accurately track access frequency
 * of pages.  But a reasonable approximation can be made to measure
 * thrashing on the inactive list, after which refaulting pages can be
 * activated optimistically to compete with the existing active pages.
 *
 * Approximating inactive page access frequency - Observations:
 *
 * 1. When a page is accessed for the first time, it is added to the
 *    head of the inactive list, slides every existing inactive page
 *    towards the tail by one slot, and pushes the current tail page
 *    out of memory.
 *
 * 2. When a page is accessed for the second time, it is promoted to
 *    the active list, shrinking the inactive list by one slot.  This
 *    also slides all inactive pages that were faulted into the cache
 *    more recently than the activated page towards the tail of the
 *    inactive list.
 *
 * Thus:
 *
 * 1. The sum of evictions and activations between any two points in
 *    time indicate the minimum number of inactive pages accessed in
 *    between.
 *
 * 2. Moving one inactive page N page slots towards the tail of the
 *    list requires at least N inactive page accesses.
 *
 * Combining these:
 *
 * 1. When a page is finally evicted from memory, the number of
 *    inactive pages accessed while the page was in cache is at least
 *    the number of page slots on the inactive list.
 *
 * 2. In addition, measuring the sum of evictions and activations (E)
 *    at the time of a page's eviction, and comparing it to another
 *    reading (R) at the time the page faults back into memory tells
 *    the minimum number of accesses while the page was not cached.
 *    This is called the refault distance.
 *
 * Because the first access of the page was the fault and the second
 * access the refault, we combine the in-cache distance with the
 * out-of-cache distance to get the complete minimum access distance
 * of this page:
 *
 *      NR_inactive + (R - E)
 *
 * And knowing the minimum access distance of a page, we can easily
 * tell if the page would be able to stay in cache assuming all page
 * slots in the cache were available:
 *
 *   NR_inactive + (R - E) <= NR_inactive + NR_active
 *
 * If we have swap we should consider about NR_inactive_anon and
 * NR_active_anon, so for page cache and anonymous respectively:
 *
 *   NR_inactive_file + (R - E) <= NR_inactive_file + NR_active_file
 *   + NR_inactive_anon + NR_active_anon
 *
 *   NR_inactive_anon + (R - E) <= NR_inactive_anon + NR_active_anon
 *   + NR_inactive_file + NR_active_file
 *
 * Which can be further simplified to:
 *
 *   (R - E) <= NR_active_file + NR_inactive_anon + NR_active_anon
 *
 *   (R - E) <= NR_active_anon + NR_inactive_file + NR_active_file
 *
 * Put into words, the refault distance (out-of-cache) can be seen as
 * a deficit in inactive list space (in-cache).  If the inactive list
 * had (R - E) more page slots, the page would not have been evicted
 * in between accesses, but activated instead.  And on a full system,
 * the only thing eating into inactive list space is active pages.
 *
 *
 *		Refaulting inactive pages
 *
 * All that is known about the active list is that the pages have been
 * accessed more than once in the past.  This means that at any given
 * time there is actually a good chance that pages on the active list
 * are no longer in active use.
 *
 * So when a refault distance of (R - E) is observed and there are at
 * least (R - E) pages in the userspace workingset, the refaulting page
 * is activated optimistically in the hope that (R - E) pages are actually
 * used less frequently than the refaulting page - or even not used at
 * all anymore.
 *
 * That means if inactive cache is refaulting with a suitable refault
 * distance, we assume the cache workingset is transitioning and put
 * pressure on the current workingset.
 *
 * If this is wrong and demotion kicks in, the pages which are truly
 * used more frequently will be reactivated while the less frequently
 * used once will be evicted from memory.
 *
 * But if this is right, the stale pages will be pushed out of memory
 * and the used pages get to stay in cache.
 *
 *		Refaulting active pages
 *
 * If on the other hand the refaulting pages have recently been
 * deactivated, it means that the active list is no longer protecting
 * actively used cache from reclaim. The cache is NOT transitioning to
 * a different workingset; the existing workingset is thrashing in the
 * space allocated to the page cache.
 *
 *
 *		Implementation
 *
 * For each node's LRU lists, a counter for inactive evictions and
 * activations is maintained (node->nonresident_age).
 *
 * On eviction, a snapshot of this counter (along with some bits to
 * identify the node) is stored in the now empty page cache
 * slot of the evicted page.  This is called a shadow entry.
 *
 * On cache misses for which there are shadow entries, an eligible(合格的)
 * refault distance will immediately activate the refaulting page.
 */

#define WORKINGSET_SHIFT 1
#define EVICTION_SHIFT	((BITS_PER_LONG - BITS_PER_XA_VALUE) +	\
			 WORKINGSET_SHIFT + NODES_SHIFT + \
			 MEM_CGROUP_ID_SHIFT)
#define EVICTION_MASK	(~0UL >> EVICTION_SHIFT)

/*
 * Eviction timestamps need to be able to cover the full range of
 * actionable refaults. However, bits are tight in the xarray
 * entry, and after storing the identifier for the lruvec there might
 * not be enough left to represent every single actionable refault. In
 * that case, we have to sacrifice granularity for distance, and group
 * evictions into coarser buckets by shaving off lower timestamp bits.
 */
static unsigned int bucket_order __read_mostly;

/**
 * pack_shadow - 将信息打包到影子条目中
 * @memcgid: 内存控制组ID
 * @pgdat: NUMA节点
 * @eviction: 驱逐时代号
 * @workingset: 是否属于工作集
 *
 * 返回值: 打包好的XA值，可以作为影子条目存储在radix树中
 *
 * 打包格式（从低位到高位）：
 * [0:0]    - workingset标志（1位）
 * [1:WORKINGSET_SHIFT-1] - NUMA节点ID
 * [WORKINGSET_SHIFT:WORKINGSET_SHIFT+NODES_SHIFT-1] - 内存控制组ID
 * [剩余高位] - 驱逐时代号
 *
 * 当page驱逐(reclaim)操作时，将nonresident_age快照信息存储在被驱逐的为空的页面缓存槽中，
 * 这种方式被称作shadow entry。通过shadow entry获取page refault distance 决定page的流转方向是active or inactive list
 *
 * 将page的Node节点、Memcg、PageWorkingset等信息组合并存储在XArray节点当中，便于后续page shadow entry的检测。
 */
static void *pack_shadow(int memcgid, pg_data_t *pgdat, unsigned long eviction,
			 bool workingset)
{
	eviction &= EVICTION_MASK;
	eviction = (eviction << MEM_CGROUP_ID_SHIFT) | memcgid;
	eviction = (eviction << NODES_SHIFT) | pgdat->node_id;
	eviction = (eviction << WORKINGSET_SHIFT) | workingset;

	return xa_mk_value(eviction);
}

/*
 * 解析shadow entry 获取page所属的Node、Memcg、PageWorkingset、eviction等信息，
 * 用于page workingset_refault的判断。
 */
static void unpack_shadow(void *shadow, int *memcgidp, pg_data_t **pgdat,
			  unsigned long *evictionp, bool *workingsetp)
{
	unsigned long entry = xa_to_value(shadow);
	int memcgid, nid;
	bool workingset;

	workingset = entry & ((1UL << WORKINGSET_SHIFT) - 1);
	entry >>= WORKINGSET_SHIFT;
	nid = entry & ((1UL << NODES_SHIFT) - 1);
	entry >>= NODES_SHIFT;
	memcgid = entry & ((1UL << MEM_CGROUP_ID_SHIFT) - 1);
	entry >>= MEM_CGROUP_ID_SHIFT;

	*memcgidp = memcgid;
	*pgdat = NODE_DATA(nid);
	*evictionp = entry;
	*workingsetp = workingset;
}

#ifdef CONFIG_LRU_GEN

static void *lru_gen_eviction(struct folio *folio)
{
	int hist;
	unsigned long token;
	unsigned long min_seq;
	struct lruvec *lruvec;
	struct lru_gen_folio *lrugen;
	int type = folio_is_file_lru(folio);
	int delta = folio_nr_pages(folio);
	int refs = folio_lru_refs(folio);
	int tier = lru_tier_from_refs(refs);
	struct mem_cgroup *memcg = folio_memcg(folio);
	struct pglist_data *pgdat = folio_pgdat(folio);

	BUILD_BUG_ON(LRU_GEN_WIDTH + LRU_REFS_WIDTH > BITS_PER_LONG - EVICTION_SHIFT);

	lruvec = mem_cgroup_lruvec(memcg, pgdat);
	lrugen = &lruvec->lrugen;
	min_seq = READ_ONCE(lrugen->min_seq[type]);
	token = (min_seq << LRU_REFS_WIDTH) | max(refs - 1, 0);

	hist = lru_hist_from_seq(min_seq);
	atomic_long_add(delta, &lrugen->evicted[hist][type][tier]);

	return pack_shadow(mem_cgroup_id(memcg), pgdat, token, refs);
}

/*
 * Tests if the shadow entry is for a folio that was recently evicted.
 * Fills in @lruvec, @token, @workingset with the values unpacked from shadow.
 */
static bool lru_gen_test_recent(void *shadow, bool file, struct lruvec **lruvec,
				unsigned long *token, bool *workingset)
{
	int memcg_id;
	unsigned long min_seq;
	struct mem_cgroup *memcg;
	struct pglist_data *pgdat;

	unpack_shadow(shadow, &memcg_id, &pgdat, token, workingset);

	memcg = mem_cgroup_from_id(memcg_id);
	*lruvec = mem_cgroup_lruvec(memcg, pgdat);

	min_seq = READ_ONCE((*lruvec)->lrugen.min_seq[file]);
	return (*token >> LRU_REFS_WIDTH) == (min_seq & (EVICTION_MASK >> LRU_REFS_WIDTH));
}

static void lru_gen_refault(struct folio *folio, void *shadow)
{
	bool recent;
	int hist, tier, refs;
	bool workingset;
	unsigned long token;
	struct lruvec *lruvec;
	struct lru_gen_folio *lrugen;
	int type = folio_is_file_lru(folio);
	int delta = folio_nr_pages(folio);

	rcu_read_lock();

	recent = lru_gen_test_recent(shadow, type, &lruvec, &token, &workingset);
	if (lruvec != folio_lruvec(folio))
		goto unlock;

	mod_lruvec_state(lruvec, WORKINGSET_REFAULT_BASE + type, delta);

	if (!recent)
		goto unlock;

	lrugen = &lruvec->lrugen;

	hist = lru_hist_from_seq(READ_ONCE(lrugen->min_seq[type]));
	/* see the comment in folio_lru_refs() */
	refs = (token & (BIT(LRU_REFS_WIDTH) - 1)) + workingset;
	tier = lru_tier_from_refs(refs);

	atomic_long_add(delta, &lrugen->refaulted[hist][type][tier]);
	mod_lruvec_state(lruvec, WORKINGSET_ACTIVATE_BASE + type, delta);

	/*
	 * Count the following two cases as stalls:
	 * 1. For pages accessed through page tables, hotter pages pushed out
	 *    hot pages which refaulted immediately.
	 * 2. For pages accessed multiple times through file descriptors,
	 *    they would have been protected by sort_folio().
	 */
	if (lru_gen_in_fault() || refs >= BIT(LRU_REFS_WIDTH) - 1) {
		set_mask_bits(&folio->flags, 0, LRU_REFS_MASK | BIT(PG_workingset));
		mod_lruvec_state(lruvec, WORKINGSET_RESTORE_BASE + type, delta);
	}
unlock:
	rcu_read_unlock();
}

#else /* !CONFIG_LRU_GEN */

static void *lru_gen_eviction(struct folio *folio)
{
	return NULL;
}

static bool lru_gen_test_recent(void *shadow, bool file, struct lruvec **lruvec,
				unsigned long *token, bool *workingset)
{
	return false;
}

static void lru_gen_refault(struct folio *folio, void *shadow)
{
}

#endif /* CONFIG_LRU_GEN */

/**
 * workingset_age_nonresident - age non-resident entries as LRU ages
 * @lruvec: the lruvec that was aged
 * @nr_pages: the number of pages to count
 *
 * As in-memory pages are aged, non-resident pages need to be aged as
 * well, in order for the refault distances later on to be comparable
 * to the in-memory dimensions. This function allows reclaim and LRU
 * operations to drive the non-resident aging along in parallel.
 */
/*
 * 当内存中的页老化时，非驻留页也需要相应老化，以便后续的refault距离
 * 与内存中的维度具有可比性。这个函数允许回收和LRU操作并行驱动非驻留老化。
 *
 * 维护一个全局的"非驻留年龄"计数器nonresident_age，每次页被驱逐时递增。
 * 这个计数器作为时间戳，用于计算页被驱逐后经过了多少"内存活动"。
 */
void workingset_age_nonresident(struct lruvec *lruvec, unsigned long nr_pages)
{
	/*
	 * Reclaiming a cgroup means reclaiming all its children in a
	 * round-robin fashion(轮询的方式). That means that each cgroup has an LRU
	 * order that is composed(组成) of the LRU orders of its child
	 * cgroups; and every page has an LRU position not just in the
	 * cgroup that owns it, but in all of that group's ancestors.
	 *
	 * So when the physical inactive list of a leaf cgroup ages,
	 * the virtual inactive lists of all its parents, including
	 * the root cgroup's, age as well.
	 */
	/* 向上遍历所有父lruvec，更新其对应的nonresident_age，原子操作避免并发 */
	do {
		atomic_long_add(nr_pages, &lruvec->nonresident_age);
	} while ((lruvec = parent_lruvec(lruvec)));
}

/**
 * workingset_eviction - note the eviction of a folio from memory
 * @target_memcg: the cgroup that is causing the reclaim
 * @folio: the folio being evicted
 *
 * Return: a shadow entry to be stored in @folio->mapping->i_pages in place
 * of the evicted @folio so that a later refault can be detected.
 */
/*
 * 记录页面被驱逐（回收）的事件
 * 通过影子条目记录页被驱逐时的age信息，用于后续refault时计算refault distance（重新访问距离），
 * 识别系统是否出现抖动。
 *
 * 1.读取当前的nonresident_age，并做右移运算；
 * 2.更新nonresident_age, 增加被驱逐页的数量；
 * 3.将特定信息打包到shadow中，包括
 *	- memcgid: 哪个cgroup的页被驱逐
 *	- pgdat: 在哪个NUMA节点
 *	- eviction: 驱逐时的时代号
 *	- workingset: 该页是否属于工作集（访问频率高）
 *
 * __remove_mapping
	-> workingset_eviction
 */
void *workingset_eviction(struct folio *folio, struct mem_cgroup *target_memcg)
{
	struct pglist_data *pgdat = folio_pgdat(folio);
	unsigned long eviction;
	struct lruvec *lruvec;
	int memcgid;

	/* Folio is fully exclusive and pins folio's memory cgroup pointer */
	VM_BUG_ON_FOLIO(folio_test_lru(folio), folio);
	VM_BUG_ON_FOLIO(folio_ref_count(folio), folio);
	VM_BUG_ON_FOLIO(!folio_test_locked(folio), folio);

	if (lru_gen_enabled())
		return lru_gen_eviction(folio);

	lruvec = mem_cgroup_lruvec(target_memcg, pgdat);
	/* XXX: target_memcg can be NULL, go through lruvec */
	memcgid = mem_cgroup_id(lruvec_memcg(lruvec));
	/* 读取nonresident_age
         * 这个计数器随着时间推移而递增，代表了"内存时代"
	 */
	eviction = atomic_long_read(&lruvec->nonresident_age);
        /*
         * 将时代号右移bucket_order位，相当于除以2^bucket_order
         * 这创建了时间桶，减少精度以节省空间，同时保持足够的时序信息
         */
	eviction >>= bucket_order;
        /*
         * 更新nonresident_age：增加被驱逐页的数量
         * 这相当于推进"内存时钟"，所有父cgroup都会更新
         */
	workingset_age_nonresident(lruvec, folio_nr_pages(folio));

	/*
         * 打包所有信息到shadow中，包括：
         *	- memcgid: 哪个cgroup的页被驱逐
         *	- pgdat: 在哪个NUMA节点
         *	- eviction: 驱逐时的时代号
         *	- workingset: 该页是否属于工作集（访问频率高）
         */
	return pack_shadow(memcgid, pgdat, eviction,
				folio_test_workingset(folio));
}

/**
 * workingset_test_recent - tests if the shadow entry is for a folio that was
 * recently evicted. Also fills in @workingset with the value unpacked from
 * shadow.
 * @shadow: the shadow entry to be tested.
 * @file: whether the corresponding folio is from the file lru.
 * @workingset: where the workingset value unpacked from shadow should
 * be stored.
 * @flush: whether to flush cgroup rstat.
 *
 * Return: true if the shadow is for a recently evicted folio; false otherwise.
 */

/*
 * 以下来自：https://zhuanlan.zhihu.com/p/10798919273
 *
 * - 当read产生一个新的page，将page放入到inactive list head头部，inactive list原有的page从head向tail尾部
 *   迁移一个位置，将原有的inactive list tail尾部的page(不是read产生的新page)驱逐出inactive list(reclaim or promoted)。
 * - 当page在inactive list第二次被访问时，将page从inactive list promoted提升到active list，对应的原有在
 *   inactive list的page从原有位置向tail尾部迁移若干位置。
 *
 * - 当一个page从inactive head到inactive tail被驱逐时，被访问过的page数至少等于inactive list的长度
 *   (就是inactive head到tail的长度即从头到尾检查的pages数)。
 * - 此外，在 page eviction驱逐时，假设测量 inactive list上的 page 被evictions驱逐与promoted提升的总和(E)，
 *   期间通过page fault重新读取的数据(R)，将E、R数据对比最小的差值即page的最小重新访问距离，该距离被称为refault distance
 *
 * page第一次访问为fault(page 存放在inactive list)，第二次被访问则是refault，
 * 因此结合内部长度与外部长度则可以得出page被访问最小的长度。
 *
 * NR_inactive + (R - E)
 * NR_inactive + (R - E)与total memory的对比即可简单的描述page是thrashing颠簸：
 *
 * NR_inactive + (R - E) < total memory, 则page thrashing，从系统的角度考虑希望保存更多的cache缓存，
 * 说明系统当中有足够的内存空间保留这些被频繁访问的页面，此时如果访问频繁的页面被频繁的换入换出显然是不合理的所以判定thrashing；
 *
 * NR_inactive + (R - E) > total memory, 则page not thrashing,说明当前系统内存不足，面对这种状态系统是无能为力的，
 * 为了保证系统的运行页面被频繁的换入换出是不可避免的，这种情况下page的流转不应该被判定为thrashing颠簸(换个角度：
 * 页面被重读的distance超过了系统total memory的长度,则认为是page的正常流转过程，不应该被判定为thrashing)；
 *
 * 通过page的访问最小距离大于inactive list且是否小于内存总和时可以判定page是否thrashing，
 * 如果大于内存总和则判定非颠簸(not thrashing)、如果小于内存总和则判定为颠簸(thrashing) 。
 *
 * NR_inactive + (R - E) <= NR_inactive + NR_active 即 (R - E) <= NR_active
 *
 * 在系统当中内存分为file-backed page、anon page根据用途的不同将page划分为文件页、匿名页。所以在计算中也要考虑其他页面内存总和数量。
 *
 * NR_inactive_file + (R - E) <= NR_inactive_file + NR_active_file + NR_inative_anon + NR_active_anon
 * NR_inactive_anon + (R - E) <= NR_inactive_file + NR_active_file + NR_inative_anon + NR_active_anon
 *
 * 进一步简化：(对应下面的代码)
 * file
 *	(R - E) <= NR_active_file + NR_inative_anon + NR_active_anon
 * anon
 *	(R - E) <= NR_inactive_file + NR_active_file + NR_active_anon
 *
 */
bool workingset_test_recent(void *shadow, bool file, bool *workingset,
				bool flush)
{

        struct mem_cgroup *eviction_memcg;  // 驱逐发生时所属的内存控制组
        struct lruvec *eviction_lruvec;     // 驱逐发生时所属的LRU向量
        unsigned long refault_distance;     // 重新访问距离，关键指标
        unsigned long workingset_size;      // 工作集大小，作为比较基准
        unsigned long refault;              // 当前的非驻留年龄（时间戳）
        int memcgid;                        // 内存控制组ID
        struct pglist_data *pgdat;          // NUMA节点
        unsigned long eviction;             // 驱逐时的非驻留年龄（时间戳）

	rcu_read_lock();

	if (lru_gen_enabled()) {
		bool recent = lru_gen_test_recent(shadow, file,
				&eviction_lruvec, &eviction, workingset);

		rcu_read_unlock();
		return recent;
	}


	/* 解压showdow，获取页面被驱逐时保存的nonresident_age, 称之为页面驱逐时间戳，其实就是页面驱逐页数 */
	unpack_shadow(shadow, &memcgid, &pgdat, &eviction, workingset);
        /* 将页面驱逐时间戳左移恢复原始精度（之前存储时右移了bucket_order位） */
	eviction <<= bucket_order;

	/*
	 * Look up the memcg associated with the stored ID. It might
	 * have been deleted since the folio's eviction.
	 *
	 * Note that in rare events the ID could have been recycled
	 * for a new cgroup that refaults a shared folio. This is
	 * impossible to tell from the available data. However, this
	 * should be a rare and limited disturbance, and activations
	 * are always speculative anyway. Ultimately, it's the aging
	 * algorithm's job to shake out the minimum access frequency
	 * for the active cache.
	 *
	 * XXX: On !CONFIG_MEMCG, this will always return NULL; it
	 * would be better if the root_mem_cgroup existed in all
	 * configurations instead.
	 */
	eviction_memcg = mem_cgroup_from_id(memcgid);
	if (!mem_cgroup_disabled() &&
	    (!eviction_memcg || !mem_cgroup_tryget(eviction_memcg))) {
		rcu_read_unlock();
		return false;
	}

	rcu_read_unlock();

	/*
	 * Flush stats (and potentially sleep) outside the RCU read section.
	 *
	 * Note that workingset_test_recent() itself might be called in RCU read
	 * section (for e.g, in cachestat) - these callers need to skip flushing
	 * stats (via the flush argument).
	 *
	 * XXX: With per-memcg flushing and thresholding, is ratelimiting
	 * still needed here?
	 */
	if (flush)
		mem_cgroup_flush_stats_ratelimited(eviction_memcg);

	/* 获取页面驱逐时对应的lruvec */
	eviction_lruvec = mem_cgroup_lruvec(eviction_memcg, pgdat);
	/* 获取当前内存节点的页面驱逐时间戳 */
	refault = atomic_long_read(&eviction_lruvec->nonresident_age);

	/*
	 * Calculate the refault distance
	 *
	 * The unsigned subtraction here gives an accurate distance
	 * across nonresident_age overflows in most cases. There is a
	 * special case: usually, shadow entries have a short lifetime
	 * and are either refaulted or reclaimed along with the inode
	 * before they get too old.  But it is not impossible for the
	 * nonresident_age to lap a shadow entry in the field, which
	 * can then result in a false small refault distance, leading
	 * to a false activation should this old entry actually
	 * refault again.  However, earlier kernels used to deactivate
	 * unconditionally with *every* reclaim invocation for the
	 * longest time, so the occasional inappropriate activation
	 * leading to pressure on the active list is not a problem.
	 */
	/* 计算refault_distance，当前的页面驱逐时间戳 - 之前的页面驱逐时间戳 */
	refault_distance = (refault - eviction) & EVICTION_MASK;

	/*
	 * Compare the distance to the existing workingset size. We
	 * don't activate pages that couldn't stay resident even if
	 * all the memory was available to the workingset. Whether
	 * workingset competition needs to consider anon or not depends
	 * on having free swap space.
	 */
	/*
	 * workingset_size计算
	 * 对于文件页
	 *	working_size = NR_active_file + NR_ative_anon + NR_inactive_anon
	 * 对于匿名页
	 *	working_size = NR_active_file + NR_inactive_file + NR_active_anon
	 */
	workingset_size = lruvec_page_state(eviction_lruvec, NR_ACTIVE_FILE);
	if (!file) {
		workingset_size += lruvec_page_state(eviction_lruvec,
						     NR_INACTIVE_FILE);
	}
	if (mem_cgroup_get_nr_swap_pages(eviction_memcg) > 0) {
		workingset_size += lruvec_page_state(eviction_lruvec,
						     NR_ACTIVE_ANON);
		if (file) {
			workingset_size += lruvec_page_state(eviction_lruvec,
						     NR_INACTIVE_ANON);
		}
	}

	mem_cgroup_put(eviction_memcg);
	/*
	 * 如果refault_distance小于或等于workingset_size，则表示是最近的重新访问,
	 * 需要重新激活该页面，否则会发生memory thrashing
	 */
	return refault_distance <= workingset_size;
}

/**
 * workingset_refault - Evaluate(评估) the refault of a previously evicted folio.
 * @folio: The freshly allocated replacement folio.
 * @shadow: Shadow entry of the evicted folio.
 *
 * Calculates and evaluates the refault distance of the previously
 * evicted folio in the context of the node and the memcg whose memory
 * pressure caused the eviction.
 */
/*
 * 评估之前被驱逐的页面的refault
 * folio: 新分配的页面
 * shadow: 之前被驱逐的页面, 被驱逐时对应的shadow
 *
 * 通过对比refault_distance和workingset_size来判断需不需要激活页面
 * 主要是在workingset_test_recent判断的
 *
 * do_page_fault
 *     -> handle_mm_fault
 *         -> __handle_mm_fault
 *		-> handle_pte_fault
 *		    -> do_swap_page
 *			-> workingset_refault
 * filemap_add_folio
 *	-> workingset_refault

 * workingset_eviction、workingset_activation 在于描述page从inactive list驱逐、升迁的过程，
 * 而workingset_refault则是发生在page fault申请page时，将页面添加到inactive or active list的过程。
 */
void workingset_refault(struct folio *folio, void *shadow)
{
	bool file = folio_is_file_lru(folio);
	struct pglist_data *pgdat;
	struct mem_cgroup *memcg;
	struct lruvec *lruvec;
	bool workingset;
	long nr;

	/* MGLRU，有自己的refault评估 */
	if (lru_gen_enabled()) {
		lru_gen_refault(folio, shadow);
		return;
	}

	/*
	 * The activation decision for this folio is made at the level
	 * where the eviction occurred, as that is where the LRU order
	 * during folio reclaim is being determined.
	 *
	 * However, the cgroup that will own the folio is the one that
	 * is actually experiencing the refault event. Make sure the folio is
	 * locked to guarantee folio_memcg() stability throughout.
	 */
	VM_BUG_ON_FOLIO(!folio_test_locked(folio), folio);
	nr = folio_nr_pages(folio);
	memcg = folio_memcg(folio);
	pgdat = folio_pgdat(folio);
	lruvec = mem_cgroup_lruvec(memcg, pgdat);	// 获取新分配页面对应的lruvec

	/* 记录workingset refault事件，/proc/vmstat: workingset_refault_anon/workingset_refault_file */
	mod_lruvec_state(lruvec, WORKINGSET_REFAULT_BASE + file, nr);

	/*
	 * 检测是否是最近重新访问，如果是返回true，继续往下走
	 *
	 * 如果refault_distance小于或等于workingset_size，则表示是最近的重新访问,
	 * 需要重新激活该页面，否则会发生memory thrashing
	 */
	if (!workingset_test_recent(shadow, file, &workingset, true))
		return;

	/* 设置PG_active */
	folio_set_active(folio);
	/* 更新nonresident_age */
	workingset_age_nonresident(lruvec, nr);
	/* 记录workingset激活事件, /proc/vmstat: workingset_activate_anon/workingset_activate_file */
	mod_lruvec_state(lruvec, WORKINGSET_ACTIVATE_BASE + file, nr);

	/* Folio was active prior to eviction */
        /* 如果folio在驱逐前是活跃的（属于工作集） */
	if (workingset) {
                /* 标记folio属于工作集 */
		folio_set_workingset(folio);
		/*
		 * XXX: Move to folio_add_lru() when it supports new vs
		 * putback
		 */
		lru_note_cost_refault(folio);
                /* 记录工作集恢复事件，/proc/vmstat: workingset_restore_anon/workingset_restore_file */
		mod_lruvec_state(lruvec, WORKINGSET_RESTORE_BASE + file, nr);
	}
}

/**
 * workingset_activation - note a page activation
 * @folio: Folio that is being activated.
 */
/*
 * 记录页面提升到active list（被激活）的事件
 *
 * folio_mark_accessed
 *	->workingset_activation
 */
void workingset_activation(struct folio *folio)
{
	struct mem_cgroup *memcg;

	rcu_read_lock();
	/*
	 * Filter non-memcg pages here, e.g. unmap can call
	 * mark_page_accessed() on VDSO pages.
	 *
	 * XXX: See workingset_refault() - this should return
	 * root_mem_cgroup even for !CONFIG_MEMCG.
	 */
	memcg = folio_memcg_rcu(folio);
	if (!mem_cgroup_disabled() && !memcg)
		goto out;
	workingset_age_nonresident(folio_lruvec(folio), folio_nr_pages(folio));
out:
	rcu_read_unlock();
}

/*
 * Shadow entries reflect the share of the working set that does not
 * fit into memory, so their number depends on the access pattern of
 * the workload.  In most cases, they will refault or get reclaimed
 * along with the inode, but a (malicious) workload that streams
 * through files with a total size several times that of available
 * memory, while preventing the inodes from being reclaimed, can
 * create excessive amounts of shadow nodes.  To keep a lid on this,
 * track shadow nodes and reclaim them when they grow way past the
 * point where they would still be useful.
 */

struct list_lru shadow_nodes;

void workingset_update_node(struct xa_node *node)
{
	struct address_space *mapping;
	struct page *page = virt_to_page(node);

	/*
	 * Track non-empty nodes that contain only shadow entries;
	 * unlink those that contain pages or are being freed.
	 *
	 * Avoid acquiring the list_lru lock when the nodes are
	 * already where they should be. The list_empty() test is safe
	 * as node->private_list is protected by the i_pages lock.
	 */
	mapping = container_of(node->array, struct address_space, i_pages);
	lockdep_assert_held(&mapping->i_pages.xa_lock);

	if (node->count && node->count == node->nr_values) {
		if (list_empty(&node->private_list)) {
			list_lru_add_obj(&shadow_nodes, &node->private_list);
			__inc_node_page_state(page, WORKINGSET_NODES);
		}
	} else {
		if (!list_empty(&node->private_list)) {
			list_lru_del_obj(&shadow_nodes, &node->private_list);
			__dec_node_page_state(page, WORKINGSET_NODES);
		}
	}
}

static unsigned long count_shadow_nodes(struct shrinker *shrinker,
					struct shrink_control *sc)
{
	unsigned long max_nodes;
	unsigned long nodes;
	unsigned long pages;

	nodes = list_lru_shrink_count(&shadow_nodes, sc);
	if (!nodes)
		return SHRINK_EMPTY;

	/*
	 * Approximate a reasonable limit for the nodes
	 * containing shadow entries. We don't need to keep more
	 * shadow entries than possible pages on the active list,
	 * since refault distances bigger than that are dismissed.
	 *
	 * The size of the active list converges toward 100% of
	 * overall page cache as memory grows, with only a tiny
	 * inactive list. Assume the total cache size for that.
	 *
	 * Nodes might be sparsely populated, with only one shadow
	 * entry in the extreme case. Obviously, we cannot keep one
	 * node for every eligible shadow entry, so compromise on a
	 * worst-case density of 1/8th. Below that, not all eligible
	 * refaults can be detected anymore.
	 *
	 * On 64-bit with 7 xa_nodes per page and 64 slots
	 * each, this will reclaim shadow entries when they consume
	 * ~1.8% of available memory:
	 *
	 * PAGE_SIZE / xa_nodes / node_entries * 8 / PAGE_SIZE
	 */
#ifdef CONFIG_MEMCG
	if (sc->memcg) {
		struct lruvec *lruvec;
		int i;

		mem_cgroup_flush_stats_ratelimited(sc->memcg);
		lruvec = mem_cgroup_lruvec(sc->memcg, NODE_DATA(sc->nid));
		for (pages = 0, i = 0; i < NR_LRU_LISTS; i++)
			pages += lruvec_page_state_local(lruvec,
							 NR_LRU_BASE + i);
		pages += lruvec_page_state_local(
			lruvec, NR_SLAB_RECLAIMABLE_B) >> PAGE_SHIFT;
		pages += lruvec_page_state_local(
			lruvec, NR_SLAB_UNRECLAIMABLE_B) >> PAGE_SHIFT;
	} else
#endif
		pages = node_present_pages(sc->nid);

	max_nodes = pages >> (XA_CHUNK_SHIFT - 3);

	if (nodes <= max_nodes)
		return 0;
	return nodes - max_nodes;
}

static enum lru_status shadow_lru_isolate(struct list_head *item,
					  struct list_lru_one *lru,
					  spinlock_t *lru_lock,
					  void *arg) __must_hold(lru_lock)
{
	struct xa_node *node = container_of(item, struct xa_node, private_list);
	struct address_space *mapping;
	int ret;

	/*
	 * Page cache insertions and deletions synchronously maintain
	 * the shadow node LRU under the i_pages lock and the
	 * lru_lock.  Because the page cache tree is emptied before
	 * the inode can be destroyed, holding the lru_lock pins any
	 * address_space that has nodes on the LRU.
	 *
	 * We can then safely transition to the i_pages lock to
	 * pin only the address_space of the particular node we want
	 * to reclaim, take the node off-LRU, and drop the lru_lock.
	 */

	mapping = container_of(node->array, struct address_space, i_pages);

	/* Coming from the list, invert the lock order */
	if (!xa_trylock(&mapping->i_pages)) {
		spin_unlock_irq(lru_lock);
		ret = LRU_RETRY;
		goto out;
	}

	/* For page cache we need to hold i_lock */
	if (mapping->host != NULL) {
		if (!spin_trylock(&mapping->host->i_lock)) {
			xa_unlock(&mapping->i_pages);
			spin_unlock_irq(lru_lock);
			ret = LRU_RETRY;
			goto out;
		}
	}

	list_lru_isolate(lru, item);
	__dec_node_page_state(virt_to_page(node), WORKINGSET_NODES);

	spin_unlock(lru_lock);

	/*
	 * The nodes should only contain one or more shadow entries,
	 * no pages, so we expect to be able to remove them all and
	 * delete and free the empty node afterwards.
	 */
	if (WARN_ON_ONCE(!node->nr_values))
		goto out_invalid;
	if (WARN_ON_ONCE(node->count != node->nr_values))
		goto out_invalid;
	xa_delete_node(node, workingset_update_node);
	__inc_lruvec_kmem_state(node, WORKINGSET_NODERECLAIM);

out_invalid:
	xa_unlock_irq(&mapping->i_pages);
	if (mapping->host != NULL) {
		if (mapping_shrinkable(mapping))
			inode_add_lru(mapping->host);
		spin_unlock(&mapping->host->i_lock);
	}
	ret = LRU_REMOVED_RETRY;
out:
	cond_resched();
	spin_lock_irq(lru_lock);
	return ret;
}

static unsigned long scan_shadow_nodes(struct shrinker *shrinker,
				       struct shrink_control *sc)
{
	/* list_lru lock nests inside the IRQ-safe i_pages lock */
	return list_lru_shrink_walk_irq(&shadow_nodes, sc, shadow_lru_isolate,
					NULL);
}

/*
 * Our list_lru->lock is IRQ-safe as it nests inside the IRQ-safe
 * i_pages lock.
 */
static struct lock_class_key shadow_nodes_key;

static int __init workingset_init(void)
{
	struct shrinker *workingset_shadow_shrinker;
	unsigned int timestamp_bits;
	unsigned int max_order;
	int ret = -ENOMEM;

	BUILD_BUG_ON(BITS_PER_LONG < EVICTION_SHIFT);
	/*
	 * Calculate the eviction bucket size to cover the longest
	 * actionable refault distance, which is currently half of
	 * memory (totalram_pages/2). However, memory hotplug may add
	 * some more pages at runtime, so keep working with up to
	 * double the initial memory by using totalram_pages as-is.
	 */
	timestamp_bits = BITS_PER_LONG - EVICTION_SHIFT;
	max_order = fls_long(totalram_pages() - 1);
	if (max_order > timestamp_bits)
		bucket_order = max_order - timestamp_bits;
	pr_info("workingset: timestamp_bits=%d max_order=%d bucket_order=%u\n",
	       timestamp_bits, max_order, bucket_order);

	workingset_shadow_shrinker = shrinker_alloc(SHRINKER_NUMA_AWARE |
						    SHRINKER_MEMCG_AWARE,
						    "mm-shadow");
	if (!workingset_shadow_shrinker)
		goto err;

	ret = __list_lru_init(&shadow_nodes, true, &shadow_nodes_key,
			      workingset_shadow_shrinker);
	if (ret)
		goto err_list_lru;

	workingset_shadow_shrinker->count_objects = count_shadow_nodes;
	workingset_shadow_shrinker->scan_objects = scan_shadow_nodes;
	/* ->count reports only fully expendable nodes */
	workingset_shadow_shrinker->seeks = 0;

	shrinker_register(workingset_shadow_shrinker);
	return 0;
err_list_lru:
	shrinker_free(workingset_shadow_shrinker);
err:
	return ret;
}
module_init(workingset_init);
