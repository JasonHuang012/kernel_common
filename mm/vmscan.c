// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 1991, 1992, 1993, 1994  Linus Torvalds
 *
 *  Swap reorganised 29.12.95, Stephen Tweedie.
 *  kswapd added: 7.1.96  sct
 *  Removed kswapd_ctl limits, and swap out as many pages as needed
 *  to bring the system back to freepages.high: 2.4.97, Rik van Riel.
 *  Zone aware kswapd started 02/00, Kanoj Sarcar (kanoj@sgi.com).
 *  Multiqueue VM started 5.8.00, Rik van Riel.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/mm.h>
#include <linux/sched/mm.h>
#include <linux/module.h>
#include <linux/gfp.h>
#include <linux/kernel_stat.h>
#include <linux/swap.h>
#include <linux/pagemap.h>
#include <linux/init.h>
#include <linux/highmem.h>
#include <linux/vmpressure.h>
#include <linux/vmstat.h>
#include <linux/file.h>
#include <linux/writeback.h>
#include <linux/blkdev.h>
#include <linux/buffer_head.h>	/* for buffer_heads_over_limit */
#include <linux/mm_inline.h>
#include <linux/backing-dev.h>
#include <linux/rmap.h>
#include <linux/topology.h>
#include <linux/cpu.h>
#include <linux/cpuset.h>
#include <linux/compaction.h>
#include <linux/notifier.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/memcontrol.h>
#include <linux/migrate.h>
#include <linux/delayacct.h>
#include <linux/sysctl.h>
#include <linux/memory-tiers.h>
#include <linux/oom.h>
#include <linux/pagevec.h>
#include <linux/prefetch.h>
#include <linux/printk.h>
#include <linux/dax.h>
#include <linux/psi.h>
#include <linux/pagewalk.h>
#include <linux/shmem_fs.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/khugepaged.h>
#include <linux/rculist_nulls.h>
#include <linux/random.h>
#include <linux/mmu_notifier.h>

#include <asm/tlbflush.h>
#include <asm/div64.h>

#include <linux/swapops.h>
#include <linux/balloon_compaction.h>
#include <linux/sched/sysctl.h>

#include "internal.h"
#include "swap.h"

#define CREATE_TRACE_POINTS
#include <trace/events/vmscan.h>

#undef CREATE_TRACE_POINTS
#include <trace/hooks/vmscan.h>

struct scan_control {
	/* How many pages shrink_list() should reclaim */
	unsigned long nr_to_reclaim;

	/*
	 * Nodemask of nodes allowed by the caller. If NULL, all nodes
	 * are scanned.
	 */
	nodemask_t	*nodemask;

	/*
	 * The memory cgroup that hit its limit and as a result is the
	 * primary target of this reclaim invocation.
	 */
	/*
	 * 和memcg相关，kswap目前可以是全局的，也可以是cgroup粒度的，
	 * 所以只需要扫描某个memcg的时候，可以指定这个变量;
	 */
	struct mem_cgroup *target_mem_cgroup;

	/*
	 * Scan pressure balancing between anon and file LRUs
	 */
	unsigned long	anon_cost;
	unsigned long	file_cost;

#ifdef CONFIG_MEMCG
	/* Swappiness value for proactive reclaim. Always use sc_swappiness()! */
	/* 主动回收的swappiness */
	int *proactive_swappiness;
#endif

	/* Can active folios be deactivated as part of reclaim? */
#define DEACTIVATE_ANON 1
#define DEACTIVATE_FILE 2
	/*
	 * deactivate，表示是否对active lru list进行扫描，
	 * 也就是是否老化active list
	 */
	/* may_deactivate，决定老化哪种类型的lru inactive list,
	 * 有两个bit, bit0决定匿名页面，bit1是文件页面,
	 * 也就是上面定义的DEACTIVATE_ANON和DEACTIVATE_FILE,
	 * may_deactivate可以等于3，也就是同时anon和file inactive链表。
	 */
	unsigned int may_deactivate:2;
	/*
	 * 是否强制老化active list, 包括anon和file，
	 * force_deactivate为1，说明当前系统内存压力比较大了, 需要提高系统内存回收能力。
	 */
	unsigned int force_deactivate:1;
	/* 不老化active链表 */
	unsigned int skipped_deactivate:1;

	/* Writepage batching in laptop mode; RECLAIM_WRITE */
	unsigned int may_writepage:1;

	/* Can mapped folios be reclaimed? */
	unsigned int may_unmap:1;

	/* Can folios be swapped as part of reclaim? */
	unsigned int may_swap:1;

	/* Not allow cache_trim_mode to be turned on as part of reclaim? */
	unsigned int no_cache_trim_mode:1;

	/* Has cache_trim_mode failed at least once? */
	unsigned int cache_trim_mode_failed:1;

	/* Proactive reclaim invoked by userspace through memory.reclaim */
	/* 用户空间触发的主动回收, 通过cgroup的memory.reclaim */
	unsigned int proactive:1;

	/*
	 * Cgroup memory below memory.low is protected as long as we
	 * don't threaten to OOM. If any cgroup is reclaimed at
	 * reduced force or passed over entirely due to its memory.low
	 * setting (memcg_low_skipped), and nothing is reclaimed as a
	 * result, then go back for one more cycle that reclaims the protected
	 * memory (memcg_low_reclaim) to avert OOM.
	 */
	/* memcg_low_reclaim为true, 表示memcg的内存使用小于low限制，也要回收内存 */
	unsigned int memcg_low_reclaim:1;
	/* memcg_low_skipped为true, 表示memcg的内存使用小于low限制，禁止回收内存 */
	unsigned int memcg_low_skipped:1;

	/* Shared cgroup tree walk failed, rescan the whole tree */
	unsigned int memcg_full_walk:1;

	unsigned int hibernation_mode:1;

	/* One of the zones is ready for compaction */
	unsigned int compaction_ready:1;

	/* There is easily reclaimable cold cache in the current node */
	/*
	 * 是否优先回收pagecache/文件页面
	 * 当系统中有大量inactive状态文件页时，尝试优先回收文件页，然后再处理匿名页
	 */
	unsigned int cache_trim_mode:1;

	/* The file folios on the current node are dangerously low */
	/*
	 * 判断文件页面状态
	 * 当系统文件页极少时，满足条件扫描平衡强制设为SCAN_ANON(get_scan_count)，
	 * 表示只扫描匿名页，平衡匿名页与文件页比例。
	 */
	unsigned int file_is_tiny:1;

	/* Always discard instead of demoting to lower tier memory */
	unsigned int no_demotion:1;

	/* Allocation order */
	s8 order;

	/* Scan (total_size >> priority) pages at once */
	s8 priority;

	/* The highest zone to isolate folios for reclaim from */
	s8 reclaim_idx;

	/* This context's GFP mask */
	gfp_t gfp_mask;

	/* Incremented by the number of inactive pages that were scanned */
	/* 已经扫描的页面数量 */
	unsigned long nr_scanned;

	/* Number of pages freed so far during a call to shrink_zones() */
	/* 已经回收的页面数量 */
	unsigned long nr_reclaimed;

	struct {
		unsigned int dirty;
		unsigned int unqueued_dirty;
		unsigned int congested;
		unsigned int writeback;
		unsigned int immediate;
		unsigned int file_taken;
		unsigned int taken;
	} nr;

	/* for recording the reclaimed slab by now */
	struct reclaim_state reclaim_state;
};

#ifdef ARCH_HAS_PREFETCHW
#define prefetchw_prev_lru_folio(_folio, _base, _field)			\
	do {								\
		if ((_folio)->lru.prev != _base) {			\
			struct folio *prev;				\
									\
			prev = lru_to_folio(&(_folio->lru));		\
			prefetchw(&prev->_field);			\
		}							\
	} while (0)
#else
#define prefetchw_prev_lru_folio(_folio, _base, _field) do { } while (0)
#endif

/*
 * From 0 .. MAX_SWAPPINESS.  Higher means more swappy.
 */
int vm_swappiness = 60;

#ifdef CONFIG_MEMCG

/* Returns true for reclaim through cgroup limits or cgroup interfaces. */
/*
 * 判断是否是在某个特定cgroup内存限制下进行的回收
 *	- 返回true，表示memcg回收
 *	- 返回false，表示全局回收
 *
 * 在特定cgroup内存限制下进行回收的场景:
 *  - 当某个 cgroup 的内存使用量接近或超过其限制时，需要在该 cgroup 内进行内存回收
 *  - 通过 cgroup 接口手动触发内存回收（比如向 memory.force_empty 写入）
 *    - 当 docker 容器内存超限时，会在该容器的 cgroup 内进行回收:
 *      echo 1 > /sys/fs/cgroup/memory/docker/<container_id>/memory.force_empty
 */
static bool cgroup_reclaim(struct scan_control *sc)
{
	/* 如果指定了目标内存控制组，就是cgroup 回收 */
	return sc->target_mem_cgroup;
}

/*
 * Returns true for reclaim on the root cgroup. This is true for direct
 * allocator reclaim and reclaim through cgroup interfaces on the root cgroup.
 */
 /*
  * 判断是否是全局回收
  *
  * 全局回收的场景:
  *	- kswapd
  *	- 直接回收中，发起内存分配的进程在root cgroup中；
  *	- root mem_cgroup内存回收；
  *	- OOM killer触发的全局回收；
  */
static bool root_reclaim(struct scan_control *sc)
{
	/* 如果没有指定cgroup，或者指定的是root mem_cgroup，则是全局回收*/
	return !sc->target_mem_cgroup || mem_cgroup_is_root(sc->target_mem_cgroup);
}

/**
 * writeback_throttling_sane - is the usual dirty throttling mechanism available?
 * @sc: scan_control in question
 *
 * The normal page dirty throttling mechanism in balance_dirty_pages() is
 * completely broken with the legacy memcg and direct stalling in
 * shrink_folio_list() is used for throttling instead, which lacks all the
 * niceties such as fairness, adaptive pausing, bandwidth proportional
 * allocation and configurability.
 *
 * This function tests whether the vmscan currently in progress can assume
 * that the normal dirty throttling mechanism is operational.
 */
static bool writeback_throttling_sane(struct scan_control *sc)
{
	if (!cgroup_reclaim(sc))
		return true;
#ifdef CONFIG_CGROUP_WRITEBACK
	if (cgroup_subsys_on_dfl(memory_cgrp_subsys))
		return true;
#endif
	return false;
}

static int sc_swappiness(struct scan_control *sc, struct mem_cgroup *memcg)
{
	if (sc->proactive && sc->proactive_swappiness)
		return *sc->proactive_swappiness;
	return mem_cgroup_swappiness(memcg);
}
#else
static bool cgroup_reclaim(struct scan_control *sc)
{
	return false;
}

static bool root_reclaim(struct scan_control *sc)
{
	return true;
}

static bool writeback_throttling_sane(struct scan_control *sc)
{
	return true;
}

static int sc_swappiness(struct scan_control *sc, struct mem_cgroup *memcg)
{
	return READ_ONCE(vm_swappiness);
}
#endif

static void set_task_reclaim_state(struct task_struct *task,
				   struct reclaim_state *rs)
{
	/* Check for an overwrite */
	WARN_ON_ONCE(rs && task->reclaim_state);

	/* Check for the nulling of an already-nulled member */
	WARN_ON_ONCE(!rs && !task->reclaim_state);

	task->reclaim_state = rs;
}

/*
 * flush_reclaim_state(): add pages reclaimed outside of LRU-based reclaim to
 * scan_control->nr_reclaimed.
 */
static void flush_reclaim_state(struct scan_control *sc)
{
	/*
	 * Currently, reclaim_state->reclaimed includes three types of pages
	 * freed outside of vmscan:
	 * (1) Slab pages.
	 * (2) Clean file pages from pruned inodes (on highmem systems).
	 * (3) XFS freed buffer pages.
	 *
	 * For all of these cases, we cannot universally link the pages to a
	 * single memcg. For example, a memcg-aware shrinker can free one object
	 * charged to the target memcg, causing an entire page to be freed.
	 * If we count the entire page as reclaimed from the memcg, we end up
	 * overestimating the reclaimed amount (potentially under-reclaiming).
	 *
	 * Only count such pages for global reclaim to prevent under-reclaiming
	 * from the target memcg; preventing unnecessary retries during memcg
	 * charging and false positives from proactive reclaim.
	 *
	 * For uncommon cases where the freed pages were actually mostly
	 * charged to the target memcg, we end up underestimating the reclaimed
	 * amount. This should be fine. The freed pages will be uncharged
	 * anyway, even if they are not counted here properly, and we will be
	 * able to make forward progress in charging (which is usually in a
	 * retry loop).
	 *
	 * We can go one step further, and report the uncharged objcg pages in
	 * memcg reclaim, to make reporting more accurate and reduce
	 * underestimation, but it's probably not worth the complexity for now.
	 */
	if (current->reclaim_state && root_reclaim(sc)) {
		sc->nr_reclaimed += current->reclaim_state->reclaimed;
		current->reclaim_state->reclaimed = 0;
	}
}

/*
 * 判断当前node是否允许内存降级，也就是能否将当前node的页面迁移到其它node
 */
static bool can_demote(int nid, struct scan_control *sc)
{
	if (!numa_demotion_enabled)
		return false;
	if (sc && sc->no_demotion)
		return false;
	if (next_demotion_node(nid) == NUMA_NO_NODE)
		return false;

	return true;
}

/*
 * 判断匿名页面是否可以回收
 * 可以回收的条件：
	- memcg为空，且swap空间还有空闲;
	- memcg不为空，且还没达到memcg的swap限制；
	- 当前node允许内存降级（demotion）;
 */
static inline bool can_reclaim_anon_pages(struct mem_cgroup *memcg,
					  int nid,
					  struct scan_control *sc)
{
	if (memcg == NULL) {
		/*
		 * For non-memcg reclaim, is there
		 * space in any swap device?
		 */
		if (get_nr_swap_pages() > 0)
			return true;
	} else {
		/* Is the memcg below its swap limit? */
		if (mem_cgroup_get_nr_swap_pages(memcg) > 0)
			return true;
	}

	/*
	 * The page can not be swapped.
	 *
	 * Can it be reclaimed from this node via demotion?
	 */
	return can_demote(nid, sc);
}

/*
 * This misses isolated folios which are not accounted for to save counters.
 * As the data only determines if reclaim or compaction continues, it is
 * not expected that isolated folios will be a dominating factor.
 */
unsigned long zone_reclaimable_pages(struct zone *zone)
{
	unsigned long nr;

	nr = zone_page_state_snapshot(zone, NR_ZONE_INACTIVE_FILE) +
		zone_page_state_snapshot(zone, NR_ZONE_ACTIVE_FILE);
	if (can_reclaim_anon_pages(NULL, zone_to_nid(zone), NULL))
		nr += zone_page_state_snapshot(zone, NR_ZONE_INACTIVE_ANON) +
			zone_page_state_snapshot(zone, NR_ZONE_ACTIVE_ANON);

	return nr;
}

/**
 * lruvec_lru_size -  Returns the number of pages on the given LRU list.
 * @lruvec: lru vector
 * @lru: lru to use
 * @zone_idx: zones to consider (use MAX_NR_ZONES - 1 for the whole LRU list)
 */
static unsigned long lruvec_lru_size(struct lruvec *lruvec, enum lru_list lru,
				     int zone_idx)
{
	unsigned long size = 0;
	int zid;

	for (zid = 0; zid <= zone_idx; zid++) {
		struct zone *zone = &lruvec_pgdat(lruvec)->node_zones[zid];

		if (!managed_zone(zone))
			continue;

		if (!mem_cgroup_disabled())
			size += mem_cgroup_get_zone_lru_size(lruvec, lru, zid);
		else
			size += zone_page_state(zone, NR_ZONE_LRU_BASE + lru);
	}
	return size;
}

static unsigned long drop_slab_node(int nid)
{
	unsigned long freed = 0;
	struct mem_cgroup *memcg = NULL;

	memcg = mem_cgroup_iter(NULL, NULL, NULL);
	do {
		freed += shrink_slab(GFP_KERNEL, nid, memcg, 0);
	} while ((memcg = mem_cgroup_iter(NULL, memcg, NULL)) != NULL);

	return freed;
}

void drop_slab(void)
{
	int nid;
	int shift = 0;
	unsigned long freed;

	do {
		freed = 0;
		for_each_online_node(nid) {
			if (fatal_signal_pending(current))
				return;

			freed += drop_slab_node(nid);
		}
	} while ((freed >> shift++) > 1);
}

static int reclaimer_offset(void)
{
	BUILD_BUG_ON(PGSTEAL_DIRECT - PGSTEAL_KSWAPD !=
			PGDEMOTE_DIRECT - PGDEMOTE_KSWAPD);
	BUILD_BUG_ON(PGSTEAL_KHUGEPAGED - PGSTEAL_KSWAPD !=
			PGDEMOTE_KHUGEPAGED - PGDEMOTE_KSWAPD);
	BUILD_BUG_ON(PGSTEAL_DIRECT - PGSTEAL_KSWAPD !=
			PGSCAN_DIRECT - PGSCAN_KSWAPD);
	BUILD_BUG_ON(PGSTEAL_KHUGEPAGED - PGSTEAL_KSWAPD !=
			PGSCAN_KHUGEPAGED - PGSCAN_KSWAPD);

	if (current_is_kswapd())
		return 0;
	if (current_is_khugepaged())
		return PGSTEAL_KHUGEPAGED - PGSTEAL_KSWAPD;
	return PGSTEAL_DIRECT - PGSTEAL_KSWAPD;
}

static inline int is_page_cache_freeable(struct folio *folio)
{
	/*
	 * A freeable page cache folio is referenced only by the caller
	 * that isolated the folio, the page cache and optional filesystem
	 * private data at folio->private.
	 */
	return folio_ref_count(folio) - folio_test_private(folio) ==
		1 + folio_nr_pages(folio);
}

/*
 * We detected a synchronous write error writing a folio out.  Probably
 * -ENOSPC.  We need to propagate that into the address_space for a subsequent
 * fsync(), msync() or close().
 *
 * The tricky part is that after writepage we cannot touch the mapping: nothing
 * prevents it from being freed up.  But we have a ref on the folio and once
 * that folio is locked, the mapping is pinned.
 *
 * We're allowed to run sleeping folio_lock() here because we know the caller has
 * __GFP_FS.
 */
static void handle_write_error(struct address_space *mapping,
				struct folio *folio, int error)
{
	folio_lock(folio);
	if (folio_mapping(folio) == mapping)
		mapping_set_error(mapping, error);
	folio_unlock(folio);
}

static bool skip_throttle_noprogress(pg_data_t *pgdat)
{
	int reclaimable = 0, write_pending = 0;
	int i;

	/*
	 * If kswapd is disabled, reschedule if necessary but do not
	 * throttle as the system is likely near OOM.
	 */
	if (pgdat->kswapd_failures >= MAX_RECLAIM_RETRIES)
		return true;

	/*
	 * If there are a lot of dirty/writeback folios then do not
	 * throttle as throttling will occur when the folios cycle
	 * towards the end of the LRU if still under writeback.
	 */
	for (i = 0; i < MAX_NR_ZONES; i++) {
		struct zone *zone = pgdat->node_zones + i;

		if (!managed_zone(zone))
			continue;

		reclaimable += zone_reclaimable_pages(zone);
		write_pending += zone_page_state_snapshot(zone,
						  NR_ZONE_WRITE_PENDING);
	}
	if (2 * write_pending <= reclaimable)
		return true;

	return false;
}

/*
 * 进入节流休眠
 * 节流会让当前回收者睡眠一段时间，等待其他回收者完成工作并被其唤醒
 *
 * 唤醒接口: wake_throttle_isolated()
 */
void reclaim_throttle(pg_data_t *pgdat, enum vmscan_throttle_state reason)
{
	wait_queue_head_t *wqh = &pgdat->reclaim_wait[reason];
	long timeout, ret;
	DEFINE_WAIT(wait);

	/*
	 * Do not throttle user workers, kthreads other than kswapd or
	 * workqueues. They may be required for reclaim to make
	 * forward progress (e.g. journalling workqueues or kthreads).
	 */
	if (!current_is_kswapd() &&
	    current->flags & (PF_USER_WORKER|PF_KTHREAD)) {
		cond_resched();
		return;
	}

	/*
	 * These figures are pulled out of thin air.
	 * VMSCAN_THROTTLE_ISOLATED is a transient condition based on too many
	 * parallel reclaimers which is a short-lived event so the timeout is
	 * short. Failing to make progress or waiting on writeback are
	 * potentially long-lived events so use a longer timeout. This is shaky
	 * logic as a failure to make progress could be due to anything from
	 * writeback to a slow device to excessive referenced folios at the tail
	 * of the inactive LRU.
	 */
	switch(reason) {
	case VMSCAN_THROTTLE_WRITEBACK:
		timeout = HZ/10;

		if (atomic_inc_return(&pgdat->nr_writeback_throttled) == 1) {
			WRITE_ONCE(pgdat->nr_reclaim_start,
				node_page_state(pgdat, NR_THROTTLED_WRITTEN));
		}

		break;
	case VMSCAN_THROTTLE_CONGESTED:
		fallthrough;
	case VMSCAN_THROTTLE_NOPROGRESS:
		if (skip_throttle_noprogress(pgdat)) {
			cond_resched();
			return;
		}

		timeout = 1;

		break;
	case VMSCAN_THROTTLE_ISOLATED:
		timeout = HZ/50;
		break;
	default:
		WARN_ON_ONCE(1);
		timeout = HZ;
		break;
	}

	prepare_to_wait(wqh, &wait, TASK_UNINTERRUPTIBLE);
	ret = schedule_timeout(timeout);
	finish_wait(wqh, &wait);

	if (reason == VMSCAN_THROTTLE_WRITEBACK)
		atomic_dec(&pgdat->nr_writeback_throttled);

	trace_mm_vmscan_throttled(pgdat->node_id, jiffies_to_usecs(timeout),
				jiffies_to_usecs(timeout - ret),
				reason);
}

/*
 * Account for folios written if tasks are throttled waiting on dirty
 * folios to clean. If enough folios have been cleaned since throttling
 * started then wakeup the throttled tasks.
 */
void __acct_reclaim_writeback(pg_data_t *pgdat, struct folio *folio,
							int nr_throttled)
{
	unsigned long nr_written;

	node_stat_add_folio(folio, NR_THROTTLED_WRITTEN);

	/*
	 * This is an inaccurate read as the per-cpu deltas may not
	 * be synchronised. However, given that the system is
	 * writeback throttled, it is not worth taking the penalty
	 * of getting an accurate count. At worst, the throttle
	 * timeout guarantees forward progress.
	 */
	nr_written = node_page_state(pgdat, NR_THROTTLED_WRITTEN) -
		READ_ONCE(pgdat->nr_reclaim_start);

	if (nr_written > SWAP_CLUSTER_MAX * nr_throttled)
		wake_up(&pgdat->reclaim_wait[VMSCAN_THROTTLE_WRITEBACK]);
}

/* possible outcome of pageout() */
typedef enum {
	/* failed to write folio out, folio is locked */
	PAGE_KEEP,
	/* move folio to the active list, folio is locked */
	PAGE_ACTIVATE,
	/* folio has been sent to the disk successfully, folio is unlocked */
	PAGE_SUCCESS,
	/* folio is clean and locked */
	PAGE_CLEAN,
} pageout_t;

/*
 * pageout is called by shrink_folio_list() for each dirty folio.
 * Calls ->writepage().
 */
static pageout_t pageout(struct folio *folio, struct address_space *mapping,
			 struct swap_iocb **plug, struct list_head *folio_list)
{
	/*
	 * If the folio is dirty, only perform writeback if that write
	 * will be non-blocking.  To prevent this allocation from being
	 * stalled by pagecache activity.  But note that there may be
	 * stalls if we need to run get_block().  We could test
	 * PagePrivate for that.
	 *
	 * If this process is currently in __generic_file_write_iter() against
	 * this folio's queue, we can perform writeback even if that
	 * will block.
	 *
	 * If the folio is swapcache, write it back even if that would
	 * block, for some throttling. This happens by accident, because
	 * swap_backing_dev_info is bust: it doesn't reflect the
	 * congestion state of the swapdevs.  Easy to fix, if needed.
	 */
	/*
	 * 检查folio是否可释放：主要检查是否有进程正在回写该页
	 * 这是为了防止回收过程被页面缓存活动阻塞。
	 *
	 * 不可释放，则保留页面，后续还是放回LRU inactive链表
	 */
	if (!is_page_cache_freeable(folio))
		return PAGE_KEEP;

	/*
	 * 处理没有地址空间映射的folio（罕见情况）。
	 * 一些数据日志系统（如journaling）可能产生这种orphaned folio：
	 * 它们没有mapping但却是脏的，同时拥有干净的缓冲区。
	 */
	if (!mapping) {
		/*
		 * Some data journaling orphaned folios can have
		 * folio->mapping == NULL while being dirty with clean buffers.
		 */
		/* 页面有对应的缓存(buffer) */
		if (folio_test_private(folio)) {
			/*
			 * 尝试释放folio对应的缓存
			 * 如果成功则清除dirty标志表示，返回PAGE_CLEAN, 后续继续回收
			 */
			if (try_to_free_buffers(folio)) {
				folio_clear_dirty(folio);
				pr_info("%s: orphaned folio\n", __func__);
				return PAGE_CLEAN;
			}
		}
		/* 无法处理，保留页面 */
		return PAGE_KEEP;
	}

	/* 没有对应的writeback接口，无法回写，只能激活 */
	if (mapping->a_ops->writepage == NULL)
		return PAGE_ACTIVATE;

	/* 清除folio dirty标志（需要同步） */
	if (folio_clear_dirty_for_io(folio)) {
		int res;
		struct writeback_control wbc = {
			.sync_mode = WB_SYNC_NONE,		// 异步writeback，不需等待
			.nr_to_write = SWAP_CLUSTER_MAX,
			.range_start = 0,			// 整个文件范围
			.range_end = LLONG_MAX,
			.for_reclaim = 1,			// 表明是回收上下文
			.swap_plug = plug,			// I/O插销，用于优化？
		};

		/*
		 * The large shmem folio can be split if CONFIG_THP_SWAP is
		 * not enabled or contiguous swap entries are failed to
		 * allocate.
		 */
		/*
		 * 特殊处理：如果是shmem大folio且不支持THP_SWAP，或者分配连续交换项失败，
		 * 可能需要分割folio。将folio_list传递给writepage，以便在需要时进行分割。
		 */
		if (shmem_mapping(mapping) && folio_test_large(folio))
			wbc.list = folio_list;

		/* 设置PG_reclaim回收标志 */
		folio_set_reclaim(folio);
		/* 调用具体的writeback接口, 待办：zram的shmem_writepage */
		res = mapping->a_ops->writepage(&folio->page, &wbc);
		if (res < 0)
			handle_write_error(mapping, folio, res);
		/* 写回过程中决定激活页面（如需要先锁定等）*/
		if (res == AOP_WRITEPAGE_ACTIVATE) {
			folio_clear_reclaim(folio);
			return PAGE_ACTIVATE;
		}

		/*
		 * 检查页面是否真的进入了写回状态。
		 * 如果没有，可能是同步写回（已完成）或者a_ops实现有问题。
		 */
		if (!folio_test_writeback(folio)) {
			/* synchronous write or broken a_ops? */
			folio_clear_reclaim(folio);
		}
		/* 记录追踪事件和统计信息 */
		trace_mm_vmscan_write_folio(folio);
		node_stat_add_folio(folio, NR_VMSCAN_WRITE);
		/* writeback成功 */
		return PAGE_SUCCESS;
	}

	return PAGE_CLEAN;
}

/*
 * Same as remove_mapping, but if the folio is removed from the mapping, it
 * gets returned with a refcount of 0.
 */
static int __remove_mapping(struct address_space *mapping, struct folio *folio,
			    bool reclaimed, struct mem_cgroup *target_memcg)
{
	int refcount;
	void *shadow = NULL;

	/* 确保folio已经锁定，并且mapping正确 */
	BUG_ON(!folio_test_locked(folio));
	BUG_ON(mapping != folio_mapping(folio));

        /*
         * 锁获取策略：
         * - 交换缓存folio不需要i_lock，因为交换缓存有自己的锁机制
         * - 普通文件缓存folio需要获取inode的i_lock来保护并发访问
         */
	if (!folio_test_swapcache(folio))
		spin_lock(&mapping->host->i_lock);

        /* 获取mapping的页缓存锁并禁用中断，防止并发修改radix树 */
	xa_lock_irq(&mapping->i_pages);
	/*
	 * The non racy check for a busy folio.
	 *
	 * Must be careful with the order of the tests. When someone has
	 * a ref to the folio, it may be possible that they dirty it then
	 * drop the reference. So if the dirty flag is tested before the
	 * refcount here, then the following race may occur:
	 *
	 * get_user_pages(&page);
	 * [user mapping goes away]
	 * write_to(page);
	 *				!folio_test_dirty(folio)    [good]
	 * folio_set_dirty(folio);
	 * folio_put(folio);
	 *				!refcount(folio)   [good, discard it]
	 *
	 * [oops, our write_to data is lost]
	 *
	 * Reversing the order of the tests ensures such a situation cannot
	 * escape unnoticed. The smp_rmb is needed to ensure the folio->flags
	 * load is not satisfied before that of folio->_refcount.
	 *
	 * Note that if the dirty flag is always set via folio_mark_dirty,
	 * and thus under the i_pages lock, then this ordering is not required.
	 */
	/*
         * 繁忙folio的非竞争检查 - 这是关键的安全检查！
         *
         * 必须注意测试顺序。当有人持有folio引用时，可能会先弄脏folio然后丢弃引用。
         * 如果在这里先测试脏标志再测试引用计数，可能会发生以下竞争：
	 *
	 *
         * 反转测试顺序可确保这种情况不会被忽视。
         * smp_rmb由folio_ref_freeze中的atomic_cmpxchg提供内存屏障。
         *
         * 注意：如果脏标志总是通过folio_mark_dirty设置（在i_pages锁下），
         * 则不需要此顺序。
         */

        /* 计算期望的引用计数：1（页缓存引用）+ 页数 */
	refcount = 1 + folio_nr_pages(folio);
        /*
         * 冻结引用计数：如果当前引用计数等于期望值，将其设置为0
         * 这是一个原子操作，提供所需的内存屏障
	 *
	 * 如果引用计数不匹配，则有其它引用这，不能free
         */
	if (!folio_ref_freeze(folio, refcount))
		goto cannot_free;

	/* note: atomic_cmpxchg in folio_ref_freeze provides the smp_rmb */
        /* 如果folio是脏的，不能安全移除，解冻引用计数 */
	if (unlikely(folio_test_dirty(folio))) {
		folio_ref_unfreeze(folio, refcount);
		goto cannot_free;
	}

	/* 根据不同的文件类型处理 */
	if (folio_test_swapcache(folio)) {
		/* 处理交换缓存页面, 需要从交换缓存中移除并释放交换槽 */
		swp_entry_t swap = folio->swap;

		/*
		 * 如果是回收 操作，并且地址空间没有退出（后续可能还会访问？）
		 * 则创建workingset shadow，用于后续检测refault和thrashing(颠簸)
		 */
		if (reclaimed && !mapping_exiting(mapping))
			shadow = workingset_eviction(folio, target_memcg);
                /*
		 * 从交换缓存的xarray中移除folio，并保存shadow
		 * (4.2内核版本之前还是用radix树, xarray是基于radix实现的)
		 */
		__delete_from_swap_cache(folio, swap, shadow);
                /* 更新内存控制组的交换统计信息 */
		mem_cgroup_swapout(folio, swap);
                /* 释放页缓存锁 */
		xa_unlock_irq(&mapping->i_pages);
                /* 释放交换槽slot的引用，如果这是最后一个引用则会真正释放交换槽 */
		put_swap_folio(folio, swap);
	} else {
		/* 处理普通文件缓存页面 */
		void (*free_folio)(struct folio *);

                /* 获取地址空间操作中定义的页面释放函数 */
		free_folio = mapping->a_ops->free_folio;
		/*
		 * Remember a shadow entry for reclaimed file cache in
		 * order to detect refaults, thus thrashing, later on.
		 *
		 * But don't store shadows in an address space that is
		 * already exiting.  This is not just an optimization,
		 * inode reclaim needs to empty out the radix tree or
		 * the nodes are lost.  Don't plant shadows behind its
		 * back.
		 *
		 * We also don't store shadows for DAX mappings because the
		 * only page cache folios found in these are zero pages
		 * covering holes, and because we don't want to mix DAX
		 * exceptional entries and shadow exceptional entries in the
		 * same address_space.
		 */
		 /*
                 * 为回收的文件缓存创建影子条目以检测refault（系统抖动）
                 *
                 * 但以下情况除外：
                 * 1. 地址空间正在退出（inode回收需要清空radix树）
                 * 2. DAX映射（不想混合DAX异常条目和影子异常条目）
                 * 3. 非文件LRU folio（如匿名内存）
                 */

		if (reclaimed && folio_is_file_lru(folio) &&
		    !mapping_exiting(mapping) && !dax_mapping(mapping))
			shadow = workingset_eviction(folio, target_memcg);
                /* 从文件缓存中移除folio */
		__filemap_remove_folio(folio, shadow);
		xa_unlock_irq(&mapping->i_pages);
                /* 如果映射可收缩，将inode添加到LRU列表以便后续回收 */
		if (mapping_shrinkable(mapping))
			inode_add_lru(mapping->host);
		spin_unlock(&mapping->host->i_lock);

                /* 如果定义了释放函数，调用它来释放folio */
		if (free_folio)
			free_folio(folio);
	}

        /* 成功移除folio，返回1 */
	return 1;

cannot_free:
	xa_unlock_irq(&mapping->i_pages);
	if (!folio_test_swapcache(folio))
		spin_unlock(&mapping->host->i_lock);
	return 0;
}

/**
 * remove_mapping() - Attempt to remove a folio from its mapping.
 * @mapping: The address space.
 * @folio: The folio to remove.
 *
 * If the folio is dirty, under writeback or if someone else has a ref
 * on it, removal will fail.
 * Return: The number of pages removed from the mapping.  0 if the folio
 * could not be removed.
 * Context: The caller should have a single refcount on the folio and
 * hold its lock.
 */
long remove_mapping(struct address_space *mapping, struct folio *folio)
{
	if (__remove_mapping(mapping, folio, false, NULL)) {
		/*
		 * Unfreezing the refcount with 1 effectively
		 * drops the pagecache ref for us without requiring another
		 * atomic operation.
		 */
		folio_ref_unfreeze(folio, 1);
		return folio_nr_pages(folio);
	}
	return 0;
}

/**
 * folio_putback_lru - Put previously isolated folio onto appropriate LRU list.
 * @folio: Folio to be returned to an LRU list.
 *
 * Add previously isolated @folio to appropriate LRU list.
 * The folio may still be unevictable for other reasons.
 *
 * Context: lru_lock must not be held, interrupts must be enabled.
 */
void folio_putback_lru(struct folio *folio)
{
	folio_add_lru(folio);
	folio_put(folio);		/* drop ref from isolate */
}

enum folio_references {
	FOLIOREF_RECLAIM,
	FOLIOREF_RECLAIM_CLEAN,
	FOLIOREF_KEEP,
	FOLIOREF_ACTIVATE,
};



/*
__alloc_pages_noprof
  -->__alloc_pages_slowpath
    --> __alloc_pages_direct_reclaim
      -->__perform_reclaim	// 直接回收
        -->try_to_free_pages	// 定义sc局部结构体
          -->do_try_to_free_pages
            --> shrink_zones [__node_reclaim] [kswapd_shrink_node] //后面这两个接口业绩会调用到shrink_node
              --> shrink_node

mem_cgroup_shrink_node
  --> shrink_lruvec
    --> lru_gen_shrink_lruvec
    --> get_scan_count [only called by shrink_list()]
    --> shrink_list



shrink_inactive_list
	shrink_folio_list

reclaim_clean_pages_from_list
	shrink_folio_list

reclaim_pages
	--> reclaim_folio_list
		--> shrink_folio_list

evict_folios
	--> shrink_folio_list

kswapd
	--> balance_pgdat
		--> kswapd_age_node
		--> memcg1_soft_limit_reclaim
		--> kswapd_shrink_node
			--> shrink_node
				--> prepare_scan_control
				--> shrink_node_memcgs
					--> shrink_lruvec
						--> lru_gen_shrink_lruvec
						--> get_scan_count [only called by shrink_list()]
						--> shrink_list
							--> shrink_inactive_list
								--> isolate_lru_folios
								--> shrink_folio_list
									--> folio_check_references
									--> try_to_unmap
									--> pageout
									--> filemap_release_folio
									--> __remove_mapping
									--> free_unref_folios
								--> move_folios_to_lru
							--> shrink_active_list
						--> shrink_inactive_list
					--> shrink_slab

*/

/*
分为两种情况：

(A).referenced_ptes等于0（说明近期该页面没有被访问过）
	1.匿名页，直接回收；
	2.文件页：
	  1.2.如果设置PG_referenced（说明该页面之前有被访问过），需要是clean状态(非脏页)，也就是如果是dirty，需要writeback之后才能回收；
	  1.1.如果没有设置PG_referenced，直接回收；

(B).referenced_ptes大于0（说明近期该页面有被访问过）
	1.匿名页，加入active list；
	2.文件页：
	  2.1.如果referenced_ptes >= 2（近期被多个进程访问过），加入active list;
	  2.2.如果referenced_ptes = 1，并且设置了PG_referenced，说明之前被访问过，最近又被访问过，加入active list;(体现第二次机会法)
	  2.3.如果referenced_ptes = 1，并且是可执行的页面(比如动态库)，加入active list;
	  2.4.如果只有referenced_ptes = 1，将其从inactive的链表尾部移动到链表头部，并设置PG_referenced；

总之，只回收引用计数为0的页面
*/
static enum folio_references folio_check_references(struct folio *folio,
						  struct scan_control *sc)
{
	int referenced_ptes, referenced_folio;
	unsigned long vm_flags;

	/*
	 * 反向映射检查，遍历所有映射该folio的进程页表，统计引用计数
	 *
	 * MGLRU: folio_referenced()-->folio_referenced_one()-->lru_gen_look_around()
	 */
	referenced_ptes = folio_referenced(folio, 1, sc->target_mem_cgroup,
					   &vm_flags);
	/*
	 * include/linux/page-flags.h:
	 *	{ return test_and_set_bit(PG_##name, folio_flags(folio, page)); }
	 *
	 * 将folio的PG_referenced清0，并返回旧值
	 * 第二次机会标志，表示folio在LRU链表上时曾被访问过
	 * referenced_folio表示曾经被访问过
	 */
	referenced_folio = folio_test_clear_referenced(folio);

	/*
	 * The supposedly reclaimable folio was found to be in a VM_LOCKED vma.
	 * Let the folio, now marked Mlocked, be moved to the unevictable list.
	 */
	/*
	 * 1. mlock页面，一般是常驻内存，不能换出，放到unevictable list(待研究：后续如何放入？)
	 *
	 * 为什么这个判断不放在最前面？ 是不是要先清除掉PG_referenced?
	 * 也不是，最新内核代码提前到folio_referenced后面了
	 * 那为什么不直接放到最前面呢？还不用遍历映射
	 */
	if (vm_flags & VM_LOCKED)
		return FOLIOREF_ACTIVATE;

	/*
	 * There are two cases to consider.
	 * 1) Rmap lock contention: rotate.
	 * 2) Skip the non-shared swapbacked folio mapped solely by
	 *    the exiting or OOM-reaped process.
	 */
	/*
	 * 锁竞争处理：-1表示获取rmap锁失败
	 * 一般发生在高并发场景，为了避免死锁，保持现状
	 *
	 * 这个应该也可以提前到folio_referenced()后面
	 * 最新代码确实是提前了，但是在VM_LOCKED判断之后
	 */
	if (referenced_ptes == -1)
		return FOLIOREF_KEEP;

	if (referenced_ptes) {
		/*
		 * All mapped folios start out with page table
		 * references from the instantiating fault, so we need
		 * to look twice if a mapped file/anon folio is used more
		 * than once.
		 *
		 * Mark it and spare it for another trip around the
		 * inactive list.  Another page table reference will
		 * lead to its activation.
		 *
		 * Note: the mark is set for activated folios as well
		 * so that recently deactivated but used folios are
		 * quickly recovered.
		 */
		/*
		 * 只要referenced_ptes大于0，都设置PG_referenced
		 * 体现第二次机会，即使这次不激活，下次也有机会
		 */
		folio_set_referenced(folio);

		/*
		 * 2. inactive->active，近期被多次访问
                 * a.之前已经被设置过PG_ref，就算这时referenced_ptes只为1，也可以加入active list；
                 * b.没有被设置过PG_referenced，但是referenced_ptes大于1，也加入active；
		 */
		if (referenced_folio || referenced_ptes > 1)
			return FOLIOREF_ACTIVATE;

		/*
		 * Activate file-backed executable folios after first usage.
		 */
		/*
		 * 3. inactive->active:
		 * 如果referenced_ptes大于0，且是可执行的file页面，比如动态库，也加入active；
		 * 原因：
		 *	代码执行通常具有局部性，很快会再次使用；
		 *	从磁盘重新加载代码的I/O代价很高；
		 */
		if ((vm_flags & VM_EXEC) && folio_is_file_lru(folio))
			return FOLIOREF_ACTIVATE;

		/*
		 * 4. 保持在inactive(移动到inactive链表头部)：referenced_ptes为1，且PG_referenced为0
		 *
		 * 如果一个文件的pagecache只被访问过一次，就都可以被加入active list，那会增加active list的压力，导致回收变慢
		 * 这种这种情况下，只是将该页面的PG_referenced置上，并且将其从inactive list的链表尾部移动到链表头部
		 * 这样的话，如果后面该页面被再次访问（referenced_ptes为1），则可以加入active list，如上面的2-a
		 */
		return FOLIOREF_KEEP;
	}

	/* Reclaim if clean, defer dirty folios to writeback */
	/*
	 * 如果只是曾经被访问过的文件页面（没有被引用）
	 * 后续判断是否是clean的，如果是则直接回收；
	 * 如果是dirty的，先写回，再回收
	 */
	if (referenced_folio && folio_is_file_lru(folio))
		return FOLIOREF_RECLAIM_CLEAN;

	/*
	 *  可以被回收:
	 *	a.没有被引用的匿名页面
	 *	b.没有被引用、且最近没被访问过的文件页面(脏页需要先写回)
	 */
	return FOLIOREF_RECLAIM;

	/*
	 * 这里会有一个疑问:
	 *	FOLIOREF_RECLAIM_CLEAN: 表示预期的干净的，可以被快速回收，fast path;
	 *	FOLIOREF_RECLAIM: 回收时如果是ditry，需要writeback再回收，slow path;
	 *
	 *	引用计数为0的情况下，为什么最近被访问的文件页面用的是FOLIOREF_RECLAIM_CLEAN，
	 *	而没被访问过的文件页面用的是FOLIOREF_RECLAIM？
	 *	最近被访问过是不是可能是写操作、变为ditry了，怎么用FOLIOREF_RECLAIM_CLEAN、预期是干净的呢？
	 *
	 * 主要是没有被引用，说明文件很可能被释放了，比如文件读取文件后关闭文件，这时页面是赶紧且可安全回收的,
	 * 就算后面页面又变脏了，skrink_folio_list中也会做双重检查
	 * 而如果是写文件，写入后页面变脏，需要写回，这时候页面还有映射，就算写回完成后，页面也可能映射着，也就是有被引用
	 *
	 * 脏页的处理：通过其他机制（如周期性写回）不会进入FOLIOREF_RECLAIM_CLEAN路径
	 *
	 * PG_referenced标志表示"曾经访问"，不表示"正在使用"
	 * 无页表引用表明资源已释放，页面可能已闲置
	 * 脏页有专门机制处理，不会混淆到干净页面路径
	 * 这种设计让内存回收器能够快速识别和回收那些短暂使用后就被放弃的干净缓存页面，从而高效地释放内存而几乎不产生I/O开销。
	 */
}

/* Check if a folio is dirty or under writeback */
static void folio_check_dirty_writeback(struct folio *folio,
				       bool *dirty, bool *writeback)
{
	struct address_space *mapping;

	/*
	 * Anonymous folios are not handled by flushers and must be written
	 * from reclaim context. Do not stall reclaim based on them.
	 * MADV_FREE anonymous folios are put into inactive file list too.
	 * They could be mistakenly treated as file lru. So further anon
	 * test is needed.
	 */
	if (!folio_is_file_lru(folio) ||
	    (folio_test_anon(folio) && !folio_test_swapbacked(folio))) {
		*dirty = false;
		*writeback = false;
		return;
	}

	/* By default assume that the folio flags are accurate */
	*dirty = folio_test_dirty(folio);
	*writeback = folio_test_writeback(folio);

	/* Verify dirty/writeback state if the filesystem supports it */
	if (!folio_test_private(folio))
		return;

	mapping = folio_mapping(folio);
	if (mapping && mapping->a_ops->is_dirty_writeback)
		mapping->a_ops->is_dirty_writeback(folio, dirty, writeback);
}

struct folio *alloc_migrate_folio(struct folio *src, unsigned long private)
{
	struct folio *dst;
	nodemask_t *allowed_mask;
	struct migration_target_control *mtc;

	mtc = (struct migration_target_control *)private;

	allowed_mask = mtc->nmask;
	/*
	 * make sure we allocate from the target node first also trying to
	 * demote or reclaim pages from the target node via kswapd if we are
	 * low on free memory on target node. If we don't do this and if
	 * we have free memory on the slower(lower) memtier, we would start
	 * allocating pages from slower(lower) memory tiers without even forcing
	 * a demotion of cold pages from the target memtier. This can result
	 * in the kernel placing hot pages in slower(lower) memory tiers.
	 */
	mtc->nmask = NULL;
	mtc->gfp_mask |= __GFP_THISNODE;
	dst = alloc_migration_target(src, (unsigned long)mtc);
	if (dst)
		return dst;

	mtc->gfp_mask &= ~__GFP_THISNODE;
	mtc->nmask = allowed_mask;

	return alloc_migration_target(src, (unsigned long)mtc);
}

/*
 * Take folios on @demote_folios and attempt to demote them to another node.
 * Folios which are not demoted are left on @demote_folios.
 */
static unsigned int demote_folio_list(struct list_head *demote_folios,
				     struct pglist_data *pgdat)
{
	int target_nid = next_demotion_node(pgdat->node_id);
	unsigned int nr_succeeded;
	nodemask_t allowed_mask;

	struct migration_target_control mtc = {
		/*
		 * Allocate from 'node', or fail quickly and quietly.
		 * When this happens, 'page' will likely just be discarded
		 * instead of migrated.
		 */
		.gfp_mask = (GFP_HIGHUSER_MOVABLE & ~__GFP_RECLAIM) | __GFP_NOWARN |
			__GFP_NOMEMALLOC | GFP_NOWAIT,
		.nid = target_nid,
		.nmask = &allowed_mask,
		.reason = MR_DEMOTION,
	};

	if (list_empty(demote_folios))
		return 0;

	if (target_nid == NUMA_NO_NODE)
		return 0;

	node_get_allowed_targets(pgdat, &allowed_mask);

	/* Demotion ignores all cpuset and mempolicy settings */
	migrate_pages(demote_folios, alloc_migrate_folio, NULL,
		      (unsigned long)&mtc, MIGRATE_ASYNC, MR_DEMOTION,
		      &nr_succeeded);

	return nr_succeeded;
}

static bool may_enter_fs(struct folio *folio, gfp_t gfp_mask)
{
	if (gfp_mask & __GFP_FS)
		return true;
	if (!folio_test_swapcache(folio) || !(gfp_mask & __GFP_IO))
		return false;
	/*
	 * We can "enter_fs" for swap-cache with only __GFP_IO
	 * providing this isn't SWP_FS_OPS.
	 * ->flags can be updated non-atomicially (scan_swap_map_slots),
	 * but that will never affect SWP_FS_OPS, so the data_race
	 * is safe.
	 */
	return !data_race(folio_swap_flags(folio) & SWP_FS_OPS);
}

/*
 * shrink_folio_list() returns the number of reclaimed pages
 */
/*
 * keep                  : 将folio放入ret_folios，后续放回LRU inctive链表;
 * keep_locked           : 先将folio解锁，将folio放入ret_folios，后续放回LRU inctive链表;
 * activate_locked       : 先将folio解锁，判断是否需要释放swap空间，非mlock页面设置PG_active，
 *                         再将folio放入ret_folios，后续放回LRU active链表;(激活/升级)
 * activate_locked_split : 先更新分割后的页面计数，后面和active_locked一样;
 *
 * shrink_folio_list流程
	- 逐步从folio_list尾部取出一个folio;
	- 尝试上锁，lock失败则保留页面，跳转到keep;
	- 如果是unevictable页面，则激活页面，跳转到active_locked;
	- 如果sc不允许解除映射，且刚好是mapped页面，也保留页面，跳转到keep_locked;
	- 如果使能了MGLRU，且是mapped页面、最近又被访问过，则保留页面，跳转keep_locked，后续MGLRU会处理;
	- 获取folio的dirty和writeback标志，更新相关统计参数;
	- 如果folio正在writeback
		- case1, 刚好是kswap流程，且有PG_reclaim标志，则激活页面，跳转到activate_locked;
		         这个folio可能因为I/O错误或者磁盘断开问题迟迟无法完成writeback ,导致无法完成回收而一直在LRU链表循环;
		- case2, 如果不是kswap流程，且没有PG_reclaim，则设置PG_reclaim，激活页面，跳转到activate_locked;
		- case3, 如果不是kswap流程，且有PG_reclaim, folio_wait_writeback()加入等待队列等待writeback完成，
		         完成后放回folio_list下个循环再尝试回收;
	- **folio_check_references()**，获取folio引用计数，返回回收策略
		- case FOLIOREF_ACTIVATE:
			mlcok页面、引用计数大于1、引用计数为1但最近被访问过的页面;
			设置PG_active，激活页面，跳转到active_loced, 最后会放回LRU active链表；
		- case FOLIOREF_KEEP:
			获取rmap锁失败、引用计数为1但最近没被访问过的页面;
			保留页面，跳转到keep_locked，最后会放回LRU inactive链表;
		- case FOLIOREF_RECLAIM:
			引用计数为0的匿名页、引用计数为0但最近没被访问过的文件页面;
			继续往下走，尝试回收;
		- case FOLIOREF_RECLAIM_CLEAN:
			引用计数为0但最近被访问过的文件页面;
			继续往下走，尝试回收;
	- 如果当前node支持降级，则将thp和大页加入降级链表demote_folios，后续处理；
	- 如果是可以swap的匿名页面，并且还没被swap, 则尝试为其分配swap空间;
		- 如果没有IO权限、或者dma pinned, 则保留页面；
		- 如果是大页，但是不能分割、或者split_folio_to_list分割失败，则激活页面；
		- add_to_swap()，为分割成功的页面分配swap空间；
		- 如果是普通页面分配swap空间失败，则跳转activate_locked_split，激活页面；
		- 如果是大页分配swap失败，再次尝试分割，分割失败跳转activate_locked;
		- 分割成功，再次尝试分配swap，如果还是失败了，还跳转activate_locked_split;
	- 如果folio是mapped的，则尝试unmap;
		- **try_to_unmap()**, 解除folio的所有映射, unmap失败则激活，跳转activate_locked;
		- 成功unmap, 则继续往下回收；
	- 如果folio是dirty的，则需要判断是否能回收，可以的回收的话需要writeback;
		- 如果是文件页面、且(不是kswap流程 或 没有PG_reclaim 或LRU链表有很多dirty页面)，则不writeback;
			- 设置PG_reclaim，激活页面，跳转activate_locked;
			- 只有kswap可以回写，不然容易栈溢出;
		- 如果是匿名页面，或者kswap流程中、有设置PG_reclaim 、LRU没有很多ditry页面的文件页面，可以writeback;
			- 如果references是FOLIOREF_RECLAIM_CLEAN，则激活页面;
			- 如果没有FS权限、或sc不支持wirteback，则保留页面;
			- 调用**pageout()**, 回写页面;
				- case PAGE_KEEP:
					如果是不可释放的页面 或者无法处理的orphand, 则保留页面；
				- case PAGE_ACTIVATE:
					如果没有对应的writeback处理接口 或 writebck过程中决定激活, 则激活页面;
				- case PAGE_SUCCESS:
					成功发起了writeback，如果writeback没完成，则保留页面，下次处理；
					如果writeback完成后有变成dirty了，还是保留页面；
					成功writeback，则继续往下回收；
	- 如果folio有对应的buffer缓存，则尝试释放；
		- 调用**filemap_release_folio()**释放缓存，失败则激活页面；
		- 释放缓存成功，判断如果没有mapping且引用计数为1（只有隔离引用），跳转free_it进行内存回收；
	- 如果是匿名页但没有swapbacked，如果引用计数为1，则继续往下回收，如果不为1，则保留页面；
	- 如果没有mapping的页面，则保留页面；
	- 如果有mappig，调用**__remove_mapping()**，从页面缓存和交换缓存中移除folio，失败则保留页面；
	- 走到这里，意味着folio已经从缓存中成功移除，可以释放了，free_it:
		- 先unlock folio;
		- 调用**folio_batch_add**，下降folio加入批量释放链表free_folios;
		- 攒够一定数量或链表满后, 调用**free_unref_folios**释放free_folios链表的页面；
		- continue处理下一个folio;
	- 遍历处理完folio_list的所有页面后，开始处理demote_folios和ret_folios链表;
	- 调用**demote_folio_list()**, 迁移需要降级的页面到其它node，对于本地node也算是回收；
	- 降级失败的页面重新放回folio_list;
	- 处理free_list没释放的页面（数量不够进行批量释放），还是调用free_unref_folios()释放；
	- 将ret_folios放回folio_list，返回后调用者会将其放回LRU链表；
	- 最后返回成功回收的页面数量；
 */
static unsigned int shrink_folio_list(struct list_head *folio_list,
		struct pglist_data *pgdat, struct scan_control *sc,
		struct reclaim_stat *stat, bool ignore_references)
{
	struct folio_batch free_folios;
	LIST_HEAD(ret_folios);		// 存放回收失败、需要放回LRU链表的页面
	LIST_HEAD(demote_folios);	// 存放回收失败、且需要降级的页面
	unsigned int nr_reclaimed = 0;	// 记录成功回收的页面数
	unsigned int pgactivate = 0;	// 记录被重新激活的页面数
	bool do_demote_pass;		// 是否允许降级
	struct swap_iocb *plug = NULL;

	folio_batch_init(&free_folios);
	memset(stat, 0, sizeof(*stat));
	cond_resched();
	/*
	 * 检查当前节点是否允许进行内存降级（demotion, 也就是能否将当前node的页面迁移到其它node），
	 * 并且扫描控制结构未明确禁止
	 */
	do_demote_pass = can_demote(pgdat->node_id, sc);

retry:
	while (!list_empty(folio_list)) {
		struct address_space *mapping;
		struct folio *folio;
		enum folio_references references = FOLIOREF_RECLAIM;
		bool dirty, writeback;
		unsigned int nr_pages;

		/* 主动调度，避免长时间占用CPU */
		cond_resched();

		/* 从folio_list尾部取出一个folio */
		folio = lru_to_folio(folio_list);
		/* 将该folio从folio_list链表删除 */
		list_del(&folio->lru);

		/*
		 * 尝试锁定该folio
		 * 如果失败说明被其它线程锁定了, 则跳过，跳转到**keep**
		 * 将其放到ret_folios链表(最后放回LRU链表)
		 */
		if (!folio_trylock(folio))
			goto keep;

		VM_BUG_ON_FOLIO(folio_test_active(folio), folio);

		nr_pages = folio_nr_pages(folio);

		/* Account the number of base pages */
		/* 更新sc->nr_scanned */
		sc->nr_scanned += nr_pages;

		/*
		 * 如果是unevictable的页面，则跳过，跳转到**activate_locked**
		 * 先判断是否需要释放其交换空间
		 * 再将folio解锁, 设置PG_active, 将其放入ret_folios(最后放到LRU active链表, 升级)
		 */
		if (unlikely(!folio_evictable(folio)))
			goto activate_locked;

		/*
		 * 和隔离页面时一样判断映射情况
		 * 如果扫描控制不允许解除映射，但此时folio有映射，跳过，跳转到**keep_locked**
		 * 将folio解锁，再将其放入ret_folios(最后放回LRU invative链表)
		 */
		if (!sc->may_unmap && folio_mapped(folio))
			goto keep_locked;

		/* folio_update_gen() tried to promote this page? */
		/*
		 * 如果使能了MGLRU + 不忽略引用计数 + folio有映射 + 最近被访问过
		 * 则跳过，保留页面，后续MGLRU会处理（待研究）
		 */
		if (lru_gen_enabled() && !ignore_references &&
		    folio_mapped(folio) && folio_test_referenced(folio))
			goto keep_locked;

		/*
		 * The number of dirty pages determines if a node is marked
		 * reclaim_congested. kswapd will stall and start writing
		 * folios if the tail of the LRU is all dirty unqueued folios.
		 */
		/*
		 * 太多diryt和writeback的页面会引起回收拥堵reclaim_congested
		 * 如果LRU链表尾部全是dirty且没排队回写的页面，kswap会被堵塞并开始回写
		 */
		folio_check_dirty_writeback(folio, &dirty, &writeback);
		if (dirty || writeback)
			stat->nr_dirty += nr_pages; // 统计脏页和回写页

		if (dirty && !writeback)
			stat->nr_unqueued_dirty += nr_pages;	// 统计ditry但未加入回写队列的页面

		/*
		 * Treat this folio as congested if folios are cycling
		 * through the LRU so quickly that the folios marked
		 * for immediate reclaim are making it to the end of
		 * the LRU a second time.
		 */
		/*
		 * 如果folio正在回写且已经被标记为回收(folio_set_reclaim()，后面的流程)
		 * 说明回收速度过快？待研究
		 */
		if (writeback && folio_test_reclaim(folio))
			stat->nr_congested += nr_pages;

		/*
		 * If a folio at the tail of the LRU is under writeback, there
		 * are three cases to consider.
		 *
		 * 1) If reclaim is encountering an excessive number
		 *    of folios under writeback and this folio has both
		 *    the writeback and reclaim flags set, then it
		 *    indicates that folios are being queued for I/O but
		 *    are being recycled through the LRU before the I/O
		 *    can complete. Waiting on the folio itself risks an
		 *    indefinite stall if it is impossible to writeback
		 *    the folio due to I/O error or disconnected storage
		 *    so instead note that the LRU is being scanned too
		 *    quickly and the caller can stall after the folio
		 *    list has been processed.
		 *
		 * 2) Global or new memcg reclaim encounters a folio that is
		 *    not marked for immediate reclaim, or the caller does not
		 *    have __GFP_FS (or __GFP_IO if it's simply going to swap,
		 *    not to fs). In this case mark the folio for immediate
		 *    reclaim and continue scanning.
		 *
		 *    Require may_enter_fs() because we would wait on fs, which
		 *    may not have submitted I/O yet. And the loop driver might
		 *    enter reclaim, and deadlock if it waits on a folio for
		 *    which it is needed to do the write (loop masks off
		 *    __GFP_IO|__GFP_FS for this reason); but more thought
		 *    would probably show more reasons.
		 *
		 * 3) Legacy memcg encounters a folio that already has the
		 *    reclaim flag set. memcg does not have any dirty folio
		 *    throttling so we could easily OOM just because too many
		 *    folios are in writeback and there is nothing else to
		 *    reclaim. Wait for the writeback to complete.
		 *
		 * In cases 1) and 2) we activate the folios to get them out of
		 * the way while we continue scanning for clean folios on the
		 * inactive list and refilling from the active list. The
		 * observation here is that waiting for disk writes is more
		 * expensive than potentially causing reloads down the line.
		 * Since they're marked for immediate reclaim, they won't put
		 * memory pressure on the cache working set any longer than it
		 * takes to write them to disk.
		 */
		if (folio_test_writeback(folio)) {
			/* Case 1 above */
			/*
			 * 在kswap流程中，如果folio正在writeback而且已经被标记为reclaim
			 * 这个folio可能因为I/O错误或者磁盘断开问题迟迟无法完成writeback
			 * 导致无法完成回收而一直在LRU链表循环
			 * 为了避免堆积太多这类页面，直接激活它
			 */
			if (current_is_kswapd() &&
			    folio_test_reclaim(folio) &&
			    test_bit(PGDAT_WRITEBACK, &pgdat->flags)) {
				stat->nr_immediate += nr_pages;		// 记录需要被立刻激活的页面数
				goto activate_locked;

			/* Case 2 above */
			/*
			 * 如果是全局回收(kswap或者直接回收)或者是新memcg回收
			 * 遇到正在writeback且还没有设置reclaim的页面
			 * 先标记reclaim标志，再放回LRU链表
			 * 可能等到下次回收，writeback就完成了，可以被回收
			 *
			 * writeback_throttling_sane(): 待研究
			 */
			} else if (writeback_throttling_sane(sc) ||
			    !folio_test_reclaim(folio) ||
			    !may_enter_fs(folio, sc->gfp_mask)) {
				/*
				 * This is slightly racy -
				 * folio_end_writeback() might have
				 * just cleared the reclaim flag, then
				 * setting the reclaim flag here ends up
				 * interpreted as the readahead flag - but
				 * that does not matter enough to care.
				 * What we do want is for this folio to
				 * have the reclaim flag set next time
				 * memcg reclaim reaches the tests above,
				 * so it will then wait for writeback to
				 * avoid OOM; and it's also appropriate
				 * in global reclaim.
				 */
				/* 设置PG_reclaim，下次遇到会等待writeback完成(else分支) */
				folio_set_reclaim(folio);
				stat->nr_writeback += nr_pages;
				goto activate_locked;

			/* Case 3 above */
			/*
			 * 如果是传统memcg
			 * 遇到正在writeback且设置了reclaim的页面，会等待writeback完成
			 * 再放回folio_list，下个循环再尝试回收
			 */
			} else {
				folio_unlock(folio);
				/* 加入等待队列，等待folio writeback完成 */
				folio_wait_writeback(folio);
				/* then go back and try same folio again */
				list_add_tail(&folio->lru, folio_list);
				continue;
			}
		}

		/*
		 * 获取folio引用计数，返回回收策略
		 */
		if (!ignore_references)
			references = folio_check_references(folio, sc);

		switch (references) {
		case FOLIOREF_ACTIVATE:
			/*
			 * mlcok页面、引用计数大于1、引用计数为1但最近被访问过的页面
			 * 设置PG_active，最后会放回LRU active链表
			 */
			goto activate_locked;
		case FOLIOREF_KEEP:
			/* 获取rmap锁失败、引用计数为1但最近没被访问过的页面
			 * 最后会放回LRU inactive链表头
			 */
			stat->nr_ref_keep += nr_pages;
			goto keep_locked;
		case FOLIOREF_RECLAIM:
			/* 引用计数为0的匿名页、引用计数为0但最近没被访问过的文件页面 */
		case FOLIOREF_RECLAIM_CLEAN:
			/* 引用计数为0但最近被访问过的文件页面, 预期为clean的页面 */
			/* 继续往下走，尝试回收 */
			; /* try to reclaim the folio below */
		}

		/*
		 * Before reclaiming the folio, try to relocate
		 * its contents to another node.
		 */
		/*
		 * 在回收之前，如果当前node是允许降级的（do_demote_pass == ture）
		 * 则尝试迁移folio到其它node，对当前node来说也算是回收释放了内存
		 *
		 * 只支持透明大页和非普通大页的页面
		 */
		if (do_demote_pass &&
		    (thp_migration_supported() || !folio_test_large(folio))) {
			/* 添加到降级链表, 后续统一处理 */
			list_add(&folio->lru, &demote_folios);
			folio_unlock(folio);
			continue;
		}

		/*
		 * Anonymous process memory has backing store?
		 * Try to allocate it some swap space here.
		 * Lazyfree folio could be freed directly
		 */
		/*
		 * PG_swapbacked表示该页面可以被swap到交换分区
		 * PG_swapcache表示该页面已经被加入swapcache
		 * 这两个标志都用在匿名页和shmem
		 * PG_swapbacked在内存回收的作用是防止数据丢失，也就是回收页面之前要swap
		 *
		 * 如果是可以swap的匿名页面，并且还没被swap, 则为其分配swap空间
		 */
		if (folio_test_anon(folio) && folio_test_swapbacked(folio)) {
			if (!folio_test_swapcache(folio)) {
				/* 没有IO权限，放回LRU invative链表 */
				if (!(sc->gfp_mask & __GFP_IO))
					goto keep_locked;
				/* dma pinned(待研究)，放回LRU invative链表 */
				if (folio_maybe_dma_pinned(folio))
					goto keep_locked;
				if (folio_test_large(folio)) {
					/* cannot split folio, skip it */
					/* 如果不能分割则激活，放到LRU active链表(是不是一直无法回收了？) */
					if (!can_split_folio(folio, 1, NULL))
						goto activate_locked;
					/*
					 * Split partially mapped folios right away.
					 * We can free the unmapped pages without IO.
					 */
					/*
					 * 分割页面, 可以先释放那些没有映射且无需IO的页面
					 * 分割失败则激活
					 */
					if (data_race(!list_empty(&folio->_deferred_list) &&
					    folio_test_partially_mapped(folio)) &&
					    split_folio_to_list(folio, folio_list))
						goto activate_locked;
				}
				/*
				 * 分割成功，则尝试分配swap空间
				 */
				if (!add_to_swap(folio)) {
					int __maybe_unused order = folio_order(folio);

					/* 如果普通页面(非大页)分配swap空间失败，则激活 */
					if (!folio_test_large(folio))
						goto activate_locked_split;
					/* Fallback to swap normal pages */
					/* 大页分割失败，则激活 */
					if (split_folio_to_list(folio, folio_list))
						goto activate_locked;
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
					if (nr_pages >= HPAGE_PMD_NR) {
						count_memcg_folio_events(folio,
							THP_SWPOUT_FALLBACK, 1);
						count_vm_event(THP_SWPOUT_FALLBACK);
					}
					count_mthp_stat(order, MTHP_STAT_SWPOUT_FALLBACK);
#endif
					/* 再次为分割后的页面分配swap空间，失败则激活 */
					if (!add_to_swap(folio))
						goto activate_locked_split;
				}
			}
		}

		/*
		 * If the folio was split above, the tail pages will make
		 * their own pass through this function and be accounted
		 * then.
		 */
		/*
		 * 前面分割了大页，则需要更新nr_scanned计数，避免重复计算
		 * 分割后的folio可能是普通页(一个页面)，也可能还是大页
		 * 因为除了首folio(本次)，其余folio会再走一次循环
		 */
		if ((nr_pages > 1) && !folio_test_large(folio)) {
			sc->nr_scanned -= (nr_pages - 1);
			nr_pages = 1;
		}

		/*
		 * The folio is mapped into the page tables of one or more
		 * processes. Try to unmap it here.
		 */
		/* 尝试unmap folio, 成功继续往下，失败则激活 */
		if (folio_mapped(folio)) {
			enum ttu_flags flags = TTU_BATCH_FLUSH;
			bool was_swapbacked = folio_test_swapbacked(folio);

			/* 如果是PMD可映射的大页，需要分割PMD(待研究) */
			if (folio_test_pmd_mappable(folio))
				flags |= TTU_SPLIT_HUGE_PMD;
			/*
			 * Without TTU_SYNC, try_to_unmap will only begin to
			 * hold PTL from the first present PTE within a large
			 * folio. Some initial PTEs might be skipped due to
			 * races with parallel PTE writes in which PTEs can be
			 * cleared temporarily before being written new present
			 * values. This will lead to a large folio is still
			 * mapped while some subpages have been partially
			 * unmapped after try_to_unmap; TTU_SYNC helps
			 * try_to_unmap acquire PTL from the first PTE,
			 * eliminating the influence of temporary PTE values.
			 */
			 /* 对于大页，设置TTU_SYNC有助于从第一个PTE开始获取PTL，消除临时PTE值的影响(new) */
			if (folio_test_large(folio))
				flags |= TTU_SYNC;

			/* **尝试解除folio的所有映射** */
			try_to_unmap(folio, flags);
			/* unmap失败，更新失败计数，并激活folio */
			if (folio_mapped(folio)) {
				stat->nr_unmap_fail += nr_pages;
				/* 原本不是swapbacked，但现在又是了 */
				if (!was_swapbacked &&
				    folio_test_swapbacked(folio))
					stat->nr_lazyfree_fail += nr_pages;
				goto activate_locked;
			}
		}

		/* 这里开始，folio是unmap的了 */

		/*
		 * Folio is unmapped now so it cannot be newly pinned anymore.
		 * No point in trying to reclaim folio if it is pinned.
		 * Furthermore we don't want to reclaim underlying fs metadata
		 * if the folio is pinned and thus potentially modified by the
		 * pinning process as that may upset the filesystem.
		 */
		/*
		 * folio现在已经没有映射了，所以它不能再被新钉住（pinned）。
		 * 如果folio可能被DMA钉住，不要回收它。
		 */
		if (folio_maybe_dma_pinned(folio))
			goto activate_locked;

		/* 获取folio的地址空间（对于文件页） */
		mapping = folio_mapping(folio);
		/* 如果folios是dirty的 */
		if (folio_test_dirty(folio)) {
			/*
			 * Only kswapd can writeback filesystem folios
			 * to avoid risk of stack overflow. But avoid
			 * injecting inefficient single-folio I/O into
			 * flusher writeback as much as possible: only
			 * write folios when we've encountered many
			 * dirty folios, and when we've already scanned
			 * the rest of the LRU for clean folios and see
			 * the same dirty folios again (with the reclaim
			 * flag set).
			 */
			/*
			 * 只有kswapd可以回写文件系统folio以避免栈溢出风险。
			 * 但尽量避免将低效的单folio I/O注入flusher回写：
			 * 只有当遇到许多脏folio，并且已经扫描完LRU其余部分寻找干净folio，
			 * 并且再次看到相同的脏folio（设置了回收标志）时，才进行回写。
			 */
			/*
			 * 如果是 dirty的文件页面 +
			 * (当前不是kswap流程 or 没有PG_reclaim标志 or 当前LRU链表尾部没有很多dirty页面)
			 * 则设置PG_reclaim标识(下次遇到再处理, 上面Case 3等wirteback完成)，并跳过，激活
			 */
			if (folio_is_file_lru(folio) &&
			    (!current_is_kswapd() ||
			     !folio_test_reclaim(folio) ||
			     !test_bit(PGDAT_DIRTY, &pgdat->flags))) {
				/*
				 * Immediately reclaim when written back.
				 * Similar in principle to folio_deactivate()
				 * except we already have the folio isolated
				 * and know it's dirty
				 */
				node_stat_mod_folio(folio, NR_VMSCAN_IMMEDIATE,
						nr_pages);
				folio_set_reclaim(folio);

				goto activate_locked;
			}

			/* 往下: 匿名页面，或者kswap流程中、有设置PG_reclaim 、LRU没有很多ditry页面的文件页面 */

			/*
			 * 如果是引用计数为0但最近被访问过的文件页面
			 * 本来预期是clean的，现在是dirty，则跳过，激活
			 */
			if (references == FOLIOREF_RECLAIM_CLEAN)
				goto keep_locked;
			/* 检查是否有FS操作权限, 因为需要往下writeback需要FS权限 */
			if (!may_enter_fs(folio, sc->gfp_mask))
				goto keep_locked;
			if (!sc->may_writepage)
				goto keep_locked;

			/*
			 * Folio is dirty. Flush the TLB if a writable entry
			 * potentially exists to avoid CPU writes after I/O
			 * starts and then write it out here.
			 */
			/* writeback之前，刷cache，确保内存的数据是最新的 */
			try_to_unmap_flush_dirty();
			/* 调用pageout()回写folio, 返回处理结果 */
			switch (pageout(folio, mapping, &plug, folio_list)) {
			case PAGE_KEEP:
				/*
				 * 不可释放的页面 或 无法处理的orphaned页面，
				 * 则保留页面，放回LRU inactive链表
				 */
				goto keep_locked;
			case PAGE_ACTIVATE:
				/*
				 * 没有对应的writeback处理接口 或 writebck过程中决定激活
				 * 则激活页面，放到LRU active链表
				 */
				/*
				 * If shmem folio is split when writeback to swap,
				 * the tail pages will make their own pass through
				 * this function and be accounted then.
				 */
				/*
				 * 如果shmem folio在回写到交换时被分割，需要重新计算
				 */
				if (nr_pages > 1 && !folio_test_large(folio)) {
					sc->nr_scanned -= (nr_pages - 1);
					nr_pages = 1;
				}
				goto activate_locked;
			case PAGE_SUCCESS:
				/* 成功发起writback, 重新计算相关统计 */
				if (nr_pages > 1 && !folio_test_large(folio)) {
					sc->nr_scanned -= (nr_pages - 1);
					nr_pages = 1;
				}
				stat->nr_pageout += nr_pages;

				/* writeback还没完成，则保留，放回LRU inactive链表，下次处理 */
				if (folio_test_writeback(folio))
					goto keep;
				/* 如果writeback完成后，又变dirty了，还是保留 */
				if (folio_test_dirty(folio))
					goto keep;

				/* 这里往下是writeback完成了，并且页面是clean的 */

				/*
				 * A synchronous write - probably a ramdisk.  Go
				 * ahead and try to reclaim the folio.
				 */
				/*
				 * 尝试加锁，加锁失败则无法回收，保留页面
				 * 待办: 前面流程中哪里folio_unlock了？
				 *       研究folio_lock具体流程
				 */
				if (!folio_trylock(folio))
					goto keep;
				/* 再次检查状态,  */
				if (folio_test_dirty(folio) ||
				    folio_test_writeback(folio))
					goto keep_locked;
				/* 重新获取mapping（可能因回写变化？） */
				mapping = folio_mapping(folio);
				fallthrough;
			case PAGE_CLEAN:
				; /* try to free the folio below */
			}
		}

		/*
		 * If the folio has buffers, try to free the buffer
		 * mappings associated with this folio. If we succeed
		 * we try to free the folio as well.
		 *
		 * We do this even if the folio is dirty.
		 * filemap_release_folio() does not perform I/O, but it
		 * is possible for a folio to have the dirty flag set,
		 * but it is actually clean (all its buffers are clean).
		 * This happens if the buffers were written out directly,
		 * with submit_bh(). ext3 will do this, as well as
		 * the blockdev mapping.  filemap_release_folio() will
		 * discover that cleanness and will drop the buffers
		 * and mark the folio clean - it can be freed.
		 *
		 * Rarely, folios can have buffers and no ->mapping.
		 * These are the folios which were not successfully
		 * invalidated in truncate_cleanup_folio().  We try to
		 * drop those buffers here and if that worked, and the
		 * folio is no longer mapped into process address space
		 * (refcount == 1) it can be freed.  Otherwise, leave
		 * the folio on the LRU so it is swappable.
		 */
		/*
		 * 如果folio有对应的buffer_head缓存，则尝试释放与其映射的buffer
		 */
		if (folio_needs_release(folio)) {
			/*
			 * 尝试释放folio对应的缓存，失败则将folio激活
			 */
			if (!filemap_release_folio(folio, sc->gfp_mask))
				goto activate_locked;
			/* 如果没有mapping且引用计数为1（只有隔离引用），可以尝试释放 */
			if (!mapping && folio_ref_count(folio) == 1) {
				folio_unlock(folio);
				/* 如果put之后引用计数为0，则释放 */
				if (folio_put_testzero(folio))
					goto free_it;
				/* 否则直接增加回收数据，后续会很快被释放？待研究 */
				else {
					/*
					 * rare race with speculative reference.
					 * the speculative reference will free
					 * this folio shortly, so we may
					 * increment nr_reclaimed here (and
					 * leave it off the LRU).
					 */
					nr_reclaimed += nr_pages;
					continue;
				}
			}
		}

		/* 处理匿名页但不是swapbacked的情况（可能是懒惰释放的folio？） */
		if (folio_test_anon(folio) && !folio_test_swapbacked(folio)) {
			/* follow __remove_mapping for reference */
			/*
			 * 尝试冻结引用计数为1，如果失败则保留页面
			 */
			if (!folio_ref_freeze(folio, 1))
				goto keep_locked;
			/*
			 * The folio has only one reference left, which is
			 * from the isolation. After the caller puts the
			 * folio back on the lru and drops the reference, the
			 * folio will be freed anyway. It doesn't matter
			 * which lru it goes on. So we don't bother checking
			 * the dirty flag here.
			 */
			/*
			 * Folio只剩下一个引用（来自隔离）。当调用者将folio放回LRU并放下引用后，
			 * folio无论如何都会被释放。它进入哪个LRU并不重要。
			 * 所以这里不检查脏标志。
			 */
			count_vm_events(PGLAZYFREED, nr_pages); // 统计懒惰释放事件
			count_memcg_folio_events(folio, PGLAZYFREED, nr_pages); // memcg统计
			count_vm_events(PGLAZYFREED, nr_pages);
			count_memcg_folio_events(folio, PGLAZYFREED, nr_pages);
		} else if (!mapping || !__remove_mapping(mapping, folio, true,
							 sc->target_mem_cgroup))
		/*
		 * 对于文件页：调用__remove_mapping从页缓存和交换缓存中移除folio，
		 * 失败则保留页面
		 */
			goto keep_locked;

		/* 走到这里，意味着folio已经从缓存中成功移除，可以释放了 */
		folio_unlock(folio);
free_it:
		/*
		 * Folio may get swapped out as a whole, need to account
		 * all pages in it.
		 */
		nr_reclaimed += nr_pages;

		/* 处理延迟分割队列（如果folio是大页的一部分） */
		folio_unqueue_deferred_split(folio);
		/* 将folio加入批量释放链表，攒够一定数量或队列满后统一释放 */
		if (folio_batch_add(&free_folios, folio) == 0) {
			/* 批量释放：memcg uncharge，刷新TLB，然后释放folio */
			mem_cgroup_uncharge_folios(&free_folios);
			try_to_unmap_flush();
			free_unref_folios(&free_folios);
		}
		/* 处理下一个folio */
		continue;

activate_locked_split:
		/*
		 * The tail pages that are failed to add into swap cache
		 * reach here.  Fixup nr_scanned and nr_pages.
		 */
		if (nr_pages > 1) {
			sc->nr_scanned -= (nr_pages - 1);
			nr_pages = 1;
		}
activate_locked:
		/* Not a candidate for swapping, so reclaim swap space. */
		/* 处理mlock页面 */
		/*
		 * 如果folio有交换缓存，而且交换分区满了 或 folio被mlock
		 * 则先释放其交换缓存空间
		 */
		if (folio_test_swapcache(folio) &&
		    (mem_cgroup_swap_full(folio) || folio_test_mlocked(folio)))
			folio_free_swap(folio);
		VM_BUG_ON_FOLIO(folio_test_active(folio), folio);
		/* 处理非mlock页面, 将其加入活跃链表 */
		if (!folio_test_mlocked(folio)) {
			int type = folio_is_file_lru(folio);
			folio_set_active(folio);
			stat->nr_activate[type] += nr_pages;
			count_memcg_folio_events(folio, PGACTIVATE, nr_pages);
		}
keep_locked:
		folio_unlock(folio);
keep:
		/* 将需要放回LRU的folio（回收失败、需激活、需保留等）加入ret_folios链表 */
		list_add(&folio->lru, &ret_folios);
		VM_BUG_ON_FOLIO(folio_test_lru(folio) ||
				folio_test_unevictable(folio), folio);
	}
	/* 'folio_list' is always empty here */

	/* Migrate folios selected for demotion */
	/*
	 * 迁移需要降级的页面，降级到其它node也算回收
	 */
	stat->nr_demoted = demote_folio_list(&demote_folios, pgdat);
	nr_reclaimed += stat->nr_demoted;
	/* Folios that could not be demoted are still in @demote_folios */
	if (!list_empty(&demote_folios)) {
		/* Folios which weren't demoted go back on @folio_list */
		list_splice_init(&demote_folios, folio_list);

		/*
		 * goto retry to reclaim the undemoted folios in folio_list if
		 * desired.
		 *
		 * Reclaiming directly from top tier nodes is not often desired
		 * due to it breaking the LRU ordering: in general memory
		 * should be reclaimed from lower tier nodes and demoted from
		 * top tier nodes.
		 *
		 * However, disabling reclaim from top tier nodes entirely
		 * would cause ooms in edge scenarios where lower tier memory
		 * is unreclaimable for whatever reason, eg memory being
		 * mlocked or too hot to reclaim. We can disable reclaim
		 * from top tier nodes in proactive reclaim though as that is
		 * not real memory pressure.
		 */
		if (!sc->proactive) {
			do_demote_pass = false;
			goto retry;
		}
	}

	/* 计算总共激活的页面数（文件+匿名） */
	pgactivate = stat->nr_activate[0] + stat->nr_activate[1];

	/* 释放批量上述释放流程剩余的folio */
	mem_cgroup_uncharge_folios(&free_folios);
	try_to_unmap_flush();
	free_unref_folios(&free_folios);

	/*
	 * 将ret_folios中的folio重新接回最初的folio_list，
	 * 让调用者（shrink_inactive_list）将它们放回LRU
	 */
	/*
	 * 将ret_folios中的folio重新放回最初的folio_list,
	 * 让调用者(shrink_inactive_list)将它们放回LRU链表
	 */
	list_splice(&ret_folios, folio_list);
	count_vm_events(PGACTIVATE, pgactivate);

	if (plug)
		swap_write_unplug(plug);
	/* 返回成功回收的页面数 */
	return nr_reclaimed;
}

unsigned int reclaim_clean_pages_from_list(struct zone *zone,
					   struct list_head *folio_list)
{
	struct scan_control sc = {
		.gfp_mask = GFP_KERNEL,
		.may_unmap = 1,
	};
	struct reclaim_stat stat;
	unsigned int nr_reclaimed;
	struct folio *folio, *next;
	LIST_HEAD(clean_folios);
	unsigned int noreclaim_flag;

	list_for_each_entry_safe(folio, next, folio_list, lru) {
		if (!folio_test_hugetlb(folio) && folio_is_file_lru(folio) &&
		    !folio_test_dirty(folio) && !__folio_test_movable(folio) &&
		    !folio_test_unevictable(folio)) {
			folio_clear_active(folio);
			list_move(&folio->lru, &clean_folios);
		}
	}

	/*
	 * We should be safe here since we are only dealing with file pages and
	 * we are not kswapd and therefore cannot write dirty file pages. But
	 * call memalloc_noreclaim_save() anyway, just in case these conditions
	 * change in the future.
	 */
	noreclaim_flag = memalloc_noreclaim_save();
	nr_reclaimed = shrink_folio_list(&clean_folios, zone->zone_pgdat, &sc,
					&stat, true);
	memalloc_noreclaim_restore(noreclaim_flag);

	list_splice(&clean_folios, folio_list);
	mod_node_page_state(zone->zone_pgdat, NR_ISOLATED_FILE,
			    -(long)nr_reclaimed);
	/*
	 * Since lazyfree pages are isolated from file LRU from the beginning,
	 * they will rotate back to anonymous LRU in the end if it failed to
	 * discard so isolated count will be mismatched.
	 * Compensate the isolated count for both LRU lists.
	 */
	mod_node_page_state(zone->zone_pgdat, NR_ISOLATED_ANON,
			    stat.nr_lazyfree_fail);
	mod_node_page_state(zone->zone_pgdat, NR_ISOLATED_FILE,
			    -(long)stat.nr_lazyfree_fail);
	return nr_reclaimed;
}

/*
 * Update LRU sizes after isolating pages. The LRU size updates must
 * be complete before mem_cgroup_update_lru_size due to a sanity check.
 */
static __always_inline void update_lru_sizes(struct lruvec *lruvec,
			enum lru_list lru, unsigned long *nr_zone_taken)
{
	int zid;

	for (zid = 0; zid < MAX_NR_ZONES; zid++) {
		if (!nr_zone_taken[zid])
			continue;

		update_lru_size(lruvec, lru, zid, -nr_zone_taken[zid]);
	}

}

/*
 * Isolating page from the lruvec to fill in @dst list by nr_to_scan times.
 *
 * lruvec->lru_lock is heavily contended.  Some of the functions that
 * shrink the lists perform better by taking out a batch of pages
 * and working on them outside the LRU lock.
 *
 * For pagecache intensive workloads, this function is the hottest
 * spot in the kernel (apart from copy_*_user functions).
 *
 * Lru_lock must be held before calling this function.
 *
 * @nr_to_scan:	The number of eligible pages to look through on the list.
 * @lruvec:	The LRU vector to pull pages from.
 * @dst:	The temp list to put pages on to.
 * @nr_scanned:	The number of pages that were scanned.
 * @sc:		The scan_control struct for this reclaim session
 * @lru:	LRU list id for isolating
 *
 * returns how many pages were moved onto *@dst.
 */
/*
 * 不参与隔离的几种页面
 * 1.所在zone编号高于本次回收规定的最高zone的页面；
 * 2.没有PG_lru标志的页面（并发隔离）；
 * 3.如果sc不允许接触映射，但仍有映射的页面；
 * 4.无法增加引用计数的页面（说明正在被释放）；
 */
static unsigned long isolate_lru_folios(unsigned long nr_to_scan,
		struct lruvec *lruvec, struct list_head *dst,
		unsigned long *nr_scanned, struct scan_control *sc,
		enum lru_list lru)
{
	struct list_head *src = &lruvec->lists[lru];	// 某个node的指定LRU链表头
	unsigned long nr_taken = 0;
	unsigned long nr_zone_taken[MAX_NR_ZONES] = { 0 };
	unsigned long nr_skipped[MAX_NR_ZONES] = { 0, };
	unsigned long skipped = 0;
	unsigned long scan, total_scan, nr_pages;
	LIST_HEAD(folios_skipped);	// 临时链表，存放跳过/不合格的folio

	total_scan = 0;
	scan = 0;
	while (scan < nr_to_scan && !list_empty(src)) {
		struct list_head *move_to = src;
		struct folio *folio;

		/* 从LRU链表尾部取出一个folio */
		folio = lru_to_folio(src);
		/* 预取下一个folio的标志位，利用CPU缓存提高性能 */
		prefetchw_prev_lru_folio(folio, src, flags);

		nr_pages = folio_nr_pages(folio);
		total_scan += nr_pages;

		/*
		 * 检查folio所在的zon是符合回收条件
		 * 如果folio所在zone编号高于本次回收规定的最高zone，则跳过folio
		 * 一般优先回收较低zone的页面
		 */
		if (folio_zonenum(folio) > sc->reclaim_idx) {
			/* 移动到skik链表，最后再重新放回LRU链表 */
			nr_skipped[folio_zonenum(folio)] += nr_pages;
			move_to = &folios_skipped;
			goto move;
		}

		/*
		 * Do not count skipped folios because that makes the function
		 * return with no isolated folios if the LRU mostly contains
		 * ineligible folios.  This causes the VM to not reclaim any
		 * folios, triggering a premature OOM.
		 * Account all pages in a folio.
		 */
		scan += nr_pages;

		/*
		 * 如果没有PG_lru标志，表示不在LRU链表中，说明有并发操作
		 *
		 * 但是为什么要跳转到move，还把它从LRU链表拿出来再放回去？(move_to为src，也就是LRU链表头)
		 * 会不会又被返回LRU链表？（实验验证一下）这样不会影响另外一个已经把它隔离出来的流程吗？
		 * **待研究**
		 */
		if (!folio_test_lru(folio))
			goto move;
		/* 如果扫描控制不允许解除映射，但folio有映射，则跳过 */
		if (!sc->may_unmap && folio_mapped(folio))
			goto move;

		/*
		 * Be careful not to clear the lru flag until after we're
		 * sure the folio is not being freed elsewhere -- the
		 * folio release code relies on it.
		 */
		/* 尝试增加引用计数，失败说明folio正在被释放 */
		if (unlikely(!folio_try_get(folio)))
			goto move;

		/*
		 * 将folio的PG_lru清0，并返回旧值
		 *
		 * 返回0，表示清除失败，已经没有PG_lru标志了
		 * 表示这个folio已经被其它线程隔离了
		 *
		 * 同上述 待研究
		 */
		if (!folio_test_clear_lru(folio)) {
			/* Another thread is already isolating this folio */
			folio_put(folio);
			goto move;
		}

		nr_taken += nr_pages;
		nr_zone_taken[folio_zonenum(folio)] += nr_pages;
		move_to = dst;
move:
		/*
		 * 将folio从LRU链表移除（进入isolate_lru_folios之前已经拿了LRU链表锁），
		 * 并放入move_to链表的头部
		 */
		list_move(&folio->lru, move_to);
	}

	/*
	 * Splice any skipped folios to the start of the LRU list. Note that
	 * this disrupts the LRU order when reclaiming for lower zones but
	 * we cannot splice to the tail. If we did then the SWAP_CLUSTER_MAX
	 * scanning would soon rescan the same folios to skip and waste lots
	 * of cpu cycles.
	 */
	if (!list_empty(&folios_skipped)) {
		int zid;

		/* 将folio_skipped链表添加到LRU链表头部 */
		list_splice(&folios_skipped, src);
		for (zid = 0; zid < MAX_NR_ZONES; zid++) {
			if (!nr_skipped[zid])
				continue;

			__count_zid_vm_events(PGSCAN_SKIP, zid, nr_skipped[zid]);
			skipped += nr_skipped[zid];
		}
	}
	*nr_scanned = total_scan;
	trace_mm_vmscan_lru_isolate(sc->reclaim_idx, sc->order, nr_to_scan,
				    total_scan, skipped, nr_taken, lru);
	/* 更新每个zone的lru size信息，/proc/zoneinfo */
	update_lru_sizes(lruvec, lru, nr_zone_taken);

	/* 返回成功隔离的页面数 */
	return nr_taken;
}

/**
 * folio_isolate_lru() - Try to isolate a folio from its LRU list.
 * @folio: Folio to isolate from its LRU list.
 *
 * Isolate a @folio from an LRU list and adjust the vmstat statistic
 * corresponding to whatever LRU list the folio was on.
 *
 * The folio will have its LRU flag cleared.  If it was found on the
 * active list, it will have the Active flag set.  If it was found on the
 * unevictable list, it will have the Unevictable flag set.  These flags
 * may need to be cleared by the caller before letting the page go.
 *
 * Context:
 *
 * (1) Must be called with an elevated refcount on the folio. This is a
 *     fundamental difference from isolate_lru_folios() (which is called
 *     without a stable reference).
 * (2) The lru_lock must not be held.
 * (3) Interrupts must be enabled.
 *
 * Return: true if the folio was removed from an LRU list.
 * false if the folio was not on an LRU list.
 */
bool folio_isolate_lru(struct folio *folio)
{
	bool ret = false;

	VM_BUG_ON_FOLIO(!folio_ref_count(folio), folio);

	if (folio_test_clear_lru(folio)) {
		struct lruvec *lruvec;

		folio_get(folio);
		lruvec = folio_lruvec_lock_irq(folio);
		lruvec_del_folio(lruvec, folio);
		unlock_page_lruvec_irq(lruvec);
		ret = true;
	}

	return ret;
}

/*
 * A direct reclaimer may isolate SWAP_CLUSTER_MAX pages from the LRU list and
 * then get rescheduled. When there are massive number of tasks doing page
 * allocation, such sleeping direct reclaimers may keep piling up on each CPU,
 * the LRU list will go small and be scanned faster than necessary, leading to
 * unnecessary swapping, thrashing and OOM.
 */
/*
 * too_many_isolated() - 检查系统中是否有过多的隔离页，防止回收过程导致系统僵住
 * @pgdat: 要检查的物理内存节点
 * @file:  要检查的页面类型（true=文件页，false=匿名页）
 * @sc:    扫描控制结构体，包含回收参数
 *
 * 返回值: true表示有过多隔离页，需要节流；false表示正常，可以继续回收
 *
 * 功能:
 * 防止直接回收者因为隔离过多页面而导致系统问题。当直接回收从LRU链表隔离大量页面后
 * 被调度出去，而系统中有大量任务在进行页面分配时，这些被隔离的页面会堆积在每个CPU上，
 * 导致LRU链表变小并被过快扫描，从而引发不必要的交换、抖动甚至OOM。
 *
 * 原理:
 * 通过比较 已隔离页面数 和 非活跃页面数 的比例来判断是否过度隔离。
 * 如果隔离的数量超过了非活跃链表的一定比例，说明回收速度跟不上分配速度，需要节流。
 *
 * 进一步说明：
 * 直接回收（Direct Reclaimer）是在分配内存时同步执行的回收过程。
 * 它们会从LRU链表隔离页面（isolate_lru_folios），然后尝试回收（shrink_folio_list）。
 * 如果在隔离后、回收前被调度出去，这些被隔离的页面就处于"既不在LRU中，也未被释放"的
 * 中间状态。如果大量回收者都这样，就会堆积大量隔离页，导致LRU链表快速变空，触发更
 * 激进的回收，形成恶性循环。
 */
static bool too_many_isolated(struct pglist_data *pgdat, int file,
		struct scan_control *sc)
{
	unsigned long inactive, isolated;
	bool too_many;

	/* kswapd是后台回收线程，不需要受到此限制 */
	if (current_is_kswapd())
		return false;
	/*
	 * 对于传统memcg，其脏页节流机制不健全，为了避免死锁，不进行过多隔离检查。
	 * 传统memcg的回收依赖于同步等待回写完成，而不是通过节流机制。
	 */
	if (!writeback_throttling_sane(sc))
		return false;

	/* 根据页面类型获取对应的非活跃页面数和已隔离页面数 */
	if (file) {
		inactive = node_page_state(pgdat, NR_INACTIVE_FILE);
		isolated = node_page_state(pgdat, NR_ISOLATED_FILE);
	} else {
		inactive = node_page_state(pgdat, NR_INACTIVE_ANON);
		isolated = node_page_state(pgdat, NR_ISOLATED_ANON);
	}

	/*
	 * GFP_NOIO/GFP_NOFS callers are allowed to isolate more pages, so they
	 * won't get blocked by normal direct-reclaimers, forming a circular
	 * deadlock.
	 */
	/*
	 * GFP_NOIO/GFP_NOFS调用者被允许隔离更多页面，这样它们不会被正常的直接回收者阻塞          * 阻塞，从而避免形成循环死锁。
	 *
	 * 原理: GFP_NOIO/GFP_NOFS分配通常来自文件系统或块设备层，这些分配可能正在等待
	 * 回收完成的页面。如果限制它们，可能导致死锁。因此给它们更高的隔离限额。
	 */
	if (gfp_has_io_fs(sc->gfp_mask))
		inactive >>= 3;

	/* 核心判断：如果已隔离页数大于非活跃页数，则认为过多 */
	too_many = isolated > inactive;

	/* Wake up tasks throttled due to too_many_isolated. */
	/* 如果没有过多隔离，唤醒可能因too_many_isolated而节流的任务 */
	/*
	 * 当函数返回 true 时，调用者（如 shrink_inactive_list）会进入
	 * reclaim_throttle(pgdat, VMSCAN_THROTTLE_ISOLATED)。
	 * 节流会让当前回收者睡眠一段时间，等待其他回收者完成工作，减少隔离页数量。
	 * 如果当前流程发现隔离页数减少（!too_many），会唤醒之前因为隔离页面过多被
	 * 节流休眠的任务，当然当前流程也继续往下走，不会被节流。
	 */
	if (!too_many)
		wake_throttle_isolated(pgdat);

	return too_many;
}

/*
 * move_folios_to_lru() moves folios from private @list to appropriate LRU list.
 *
 * Returns the number of pages moved to the given lruvec.
 */
/*
 * 将list链表的页面直接加入LRU链表，而不是先加到cpu缓存
 *
 * 1.如果是unevictable页面直接加到LRU unevictable链表;
 * 2.将folio引用计数减1，如果减去后为0，则直接释放(批量释放);
 * 3.**lruvec_add_folio()**，将folio加入对应的LRU链表;
 */
static unsigned int move_folios_to_lru(struct lruvec *lruvec,
		struct list_head *list)
{
	int nr_pages, nr_moved = 0;
	struct folio_batch free_folios;

	folio_batch_init(&free_folios);
	while (!list_empty(list)) {
		/* 从要加入LRU链表的链表尾部取出一个folio */
		struct folio *folio = lru_to_folio(list);

		VM_BUG_ON_FOLIO(folio_test_lru(folio), folio);
		/* 将folio从原来的链表删除 */
		list_del(&folio->lru);
		if (unlikely(!folio_evictable(folio))) {
			spin_unlock_irq(&lruvec->lru_lock);
			folio_putback_lru(folio);
			spin_lock_irq(&lruvec->lru_lock);
			continue;
		}

		/*
		 * The folio_set_lru needs to be kept here for list integrity.
		 * Otherwise:
		 *   #0 move_folios_to_lru             #1 release_pages
		 *   if (!folio_put_testzero())
		 *				      if (folio_put_testzero())
		 *				        !lru //skip lru_lock
		 *     folio_set_lru()
		 *     list_add(&folio->lru,)
		 *                                        list_add(&folio->lru,)
		 */
		folio_set_lru(folio);

		/*
		 * 如果folio只有一个引用计数，那put之后可以直接释放
		 * 将这个folio加入free_folios缓存，如果缓存满了，则直接释放
		 */
		if (unlikely(folio_put_testzero(folio))) {
			__folio_clear_lru_flags(folio);

			folio_unqueue_deferred_split(folio);
			if (folio_batch_add(&free_folios, folio) == 0) {
				spin_unlock_irq(&lruvec->lru_lock);
				mem_cgroup_uncharge_folios(&free_folios);
				free_unref_folios(&free_folios);
				spin_lock_irq(&lruvec->lru_lock);
			}

			continue;
		}

		/*
		 * All pages were isolated from the same lruvec (and isolation
		 * inhibits memcg migration).
		 */
		VM_BUG_ON_FOLIO(!folio_matches_lruvec(folio, lruvec), folio);
		/* 将folio直接加入对应的LRU链表，而不是percpu缓存 */
		lruvec_add_folio(lruvec, folio);
		nr_pages = folio_nr_pages(folio);
		nr_moved += nr_pages;
		if (folio_test_active(folio))
			workingset_age_nonresident(lruvec, nr_pages);
	}

	/* 继续释放剩余的需要释放的folio */
	if (free_folios.nr) {
		spin_unlock_irq(&lruvec->lru_lock);
		mem_cgroup_uncharge_folios(&free_folios);
		free_unref_folios(&free_folios);
		spin_lock_irq(&lruvec->lru_lock);
	}

	return nr_moved;
}

/*
 * If a kernel thread (such as nfsd for loop-back mounts) services a backing
 * device by writing to the page cache it sets PF_LOCAL_THROTTLE. In this case
 * we should not throttle.  Otherwise it is safe to do so.
 */
static int current_may_throttle(void)
{
	return !(current->flags & PF_LOCAL_THROTTLE);
}

/*
 * shrink_inactive_list() is a helper for shrink_node().  It returns the number
 * of reclaimed pages
 */
/*
 * 从LRU inactive链表中回收合适的页面
 *
 * shrink_inactive_list流程:
 *	- **too_many_isolated()**, 判断是否存在太多隔离页面，
 *		- 如果隔离页面数过多，则调用**reclaim_throttle()**休眠等待一次；
 *		- 如果隔离页面数正常, 则继续往下；
 *	- **lru_add_drain()**, 将cpu缓存的页面加入LRU链表；
 *	- **isolate_lru_folios**，从LRU inactive链表的尾部隔离出指定数量的页面；
 *	- **shrink_folio_list()**，尝试回收隔离出来的页面，返回成功回收的页面数量；
 *	- **move_folios_to_lru()**, 将回收失败的页面重新放回LRU链表，
 *		- 将folio直接加入对应的LRU链表(不是percpu缓存)，如果引用计数为1的, 则直接释放；
 *	- 如果隔离出来的页面都是dirty但是没有加入writeback，则唤醒回写线程
 */
static unsigned long shrink_inactive_list(unsigned long nr_to_scan,
		struct lruvec *lruvec, struct scan_control *sc,
		enum lru_list lru)
{
	LIST_HEAD(folio_list);
	unsigned long nr_scanned;
	unsigned int nr_reclaimed = 0;	// 成功回收的页面数量
	unsigned long nr_taken;		// 从LRU链表取出的页面数量
	struct reclaim_stat stat;
	bool file = is_file_lru(lru);
	enum vm_event_item item;
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);
	bool stalled = false;

	/*
	 * 判断是否存在太多隔离页面，
	 * 如果隔离页面不多，则唤醒之前因为隔离页面过多而休眠的线程，当前流程也继续往下走
	 *
	 * 隔离页面越多，不活跃LRU链表页面越少，容易引起系统问题
	 * LRU链表越短、扫描越快，越容易造成激进的回收，引发不必要的交换、抖动甚至OOM
	 */
	while (unlikely(too_many_isolated(pgdat, file, sc))) {
		/*
		 * 只节流休眠等待一次，如果隔离页面满足条件被其它流程唤醒后，
		 * 又多了很多隔离页面，导致隔离页面比例又过高，就直接返回了
		 */
		if (stalled)
			return 0;

		/* wait a bit for the reclaimer. */
		stalled = true;
		/*
		 * 如果隔离页面比例过高，则进入节流休眠，等到当隔离页面减少时被其它流程唤醒
		 * 在too_many_isolated()中被唤醒
		 */
		reclaim_throttle(pgdat, VMSCAN_THROTTLE_ISOLATED);

		/* We are about to die and free our memory. Return now. */
		if (fatal_signal_pending(current))
			return SWAP_CLUSTER_MAX;
	}

	/*
	 * 将CPU缓存的LRU页面更新到对应的LRU链表中
	 */
	lru_add_drain();

	/* 隔离之前拿LRU锁 */
	spin_lock_irq(&lruvec->lru_lock);

	/*
	 * 从LRU链表中隔离出指定数量的页面, 放到folio_list中
	 *
	 * nr_taken: 实际隔离出的页面数量
	 * nr_scanned: 实际扫描的总页面数量（包含不合格的页面）
	 *
	 * 不参与隔离的几种页面
	 * 1.所在zone编号高于本次回收规定的最高zone的页面；
	 * 2.没有PG_lru标志的页面（并发隔离）；
	 * 3.如果sc不允许接触映射，但仍有映射的页面；
	 * 4.无法增加引用计数的页面（说明正在被释放）；
	 */
	nr_taken = isolate_lru_folios(nr_to_scan, lruvec, &folio_list,
				     &nr_scanned, sc, lru);

	/* 更新节点的隔离页面数量 */
	__mod_node_page_state(pgdat, NR_ISOLATED_ANON + file, nr_taken);
	/* 记录扫描事件，需要区分是kswap还是直接回收 */
	item = PGSCAN_KSWAPD + reclaimer_offset();
	/* 全局回收才记录全局事件 */
	if (!cgroup_reclaim(sc))
		__count_vm_events(item, nr_scanned);
	/* 记录memcg相关的扫描事件 */
	__count_memcg_events(lruvec_memcg(lruvec), item, nr_scanned);
	/* 记录匿名/文件页的扫描事件 */
	__count_vm_events(PGSCAN_ANON + file, nr_scanned);

	/* 释放LRU锁 */
	spin_unlock_irq(&lruvec->lru_lock);

	/* 如果没有成功隔离出任何页面，直接返回 */
	if (nr_taken == 0)
		return 0;

	/*
	 * **回收核心**
	 * 尝试回收隔离出来的页面，返回回收成功的页面数量
	 * stat记录回收相关信息
	 */
	nr_reclaimed = shrink_folio_list(&folio_list, pgdat, sc, &stat, false);

	spin_lock_irq(&lruvec->lru_lock);
	/*
	 * 将回收失败的页面重新放回LRU链表(而不是先加到percpu缓存),
	 * 激活的放active链表，保留的还是放inactive链表
	 * 引用计数为1的folio，则直接释放
	 */
	move_folios_to_lru(lruvec, &folio_list);

	__mod_lruvec_state(lruvec, PGDEMOTE_KSWAPD + reclaimer_offset(),
					stat.nr_demoted);
	/* 再次更新节点的隔离页面数量 */
	__mod_node_page_state(pgdat, NR_ISOLATED_ANON + file, -nr_taken);
	/* 记录回收成功事件: 区分kswapd和直接回收 */
	item = PGSTEAL_KSWAPD + reclaimer_offset();
	if (!cgroup_reclaim(sc))
		__count_vm_events(item, nr_reclaimed);
	__count_memcg_events(lruvec_memcg(lruvec), item, nr_reclaimed);
	/* 记录匿名/文件页的回收事件 */
	__count_vm_events(PGSTEAL_ANON + file, nr_reclaimed);
	spin_unlock_irq(&lruvec->lru_lock);

	/*
	 * 计算此次回收的成本、效率，影响后续扫描优先级
	 * nr_pageout: 回收过程中writebate的页面数
	 * nr_scanned - nr_reclaimed: 扫描但未被回收的页面数
	 */
	lru_note_cost(lruvec, file, stat.nr_pageout, nr_scanned - nr_reclaimed);

	/*
	 * If dirty folios are scanned that are not queued for IO, it
	 * implies that flushers are not doing their job. This can
	 * happen when memory pressure pushes dirty folios to the end of
	 * the LRU before the dirty limits are breached and the dirty
	 * data has expired. It can also happen when the proportion of
	 * dirty folios grows not through writes but through memory
	 * pressure reclaiming all the clean cache. And in some cases,
	 * the flushers simply cannot keep up with the allocation
	 * rate. Nudge the flusher threads in case they are asleep.
	 */
	/*
	 * 待研究
	 * 如果扫描的脏页没有加入回写队列，意味着flusher线程没有正常工作。
	 * 这可能发生在：
	 * 1. 内存压力在脏数据限制被突破和脏数据过期之前，就将脏页推到了LRU末尾
	 * 2. 脏页比例的增长不是通过写入，而是通过内存压力回收所有干净缓存导致的
	 * 3. flusher线程根本无法跟上分配速率
	 * 在这种情况下，唤醒flusher线程。
	 */
	/* 隔离出来的页面都是 dirtyr但还没加入回写队列的页面 (dirty && !writeback) */
	if (stat.nr_unqueued_dirty == nr_taken) {
		/* 唤醒回写线程 */
		wakeup_flusher_threads(WB_REASON_VMSCAN);
		/*
		 * For cgroupv1 dirty throttling is achieved by waking up
		 * the kernel flusher here and later waiting on folios
		 * which are in writeback to finish (see shrink_folio_list()).
		 *
		 * Flusher may not be able to issue writeback quickly
		 * enough for cgroupv1 writeback throttling to work
		 * on a large system.
		 */
		if (!writeback_throttling_sane(sc))
			reclaim_throttle(pgdat, VMSCAN_THROTTLE_WRITEBACK);
	}

	/* 将本次回收的统计信息累加到扫描控制结构中 */
	sc->nr.dirty += stat.nr_dirty;
	sc->nr.congested += stat.nr_congested;
	sc->nr.unqueued_dirty += stat.nr_unqueued_dirty;
	sc->nr.writeback += stat.nr_writeback;
	sc->nr.immediate += stat.nr_immediate;
	sc->nr.taken += nr_taken;
	if (file)
		sc->nr.file_taken += nr_taken;

	/* 记录跟踪事件，便于调试和性能分析 */
	trace_mm_vmscan_lru_shrink_inactive(pgdat->node_id,
			nr_scanned, nr_reclaimed, &stat, sc->priority, file);

	/* 返回成功回收的页面数量 */
	return nr_reclaimed;
}

/*
 * shrink_active_list() moves folios from the active LRU to the inactive LRU.
 *
 * We move them the other way if the folio is referenced by one or more
 * processes.
 *
 * If the folios are mostly unmapped, the processing is fast and it is
 * appropriate to hold lru_lock across the whole operation.  But if
 * the folios are mapped, the processing is slow (folio_referenced()), so
 * we should drop lru_lock around each folio.  It's impossible to balance
 * this, so instead we remove the folios from the LRU while processing them.
 * It is safe to rely on the active flag against the non-LRU folios in here
 * because nobody will play with that bit on a non-LRU folio.
 *
 * The downside is that we have to touch folio->_refcount against each folio.
 * But we had to alter folio->flags anyway.
 */
/*
 * 从LRU active链表中取出合适的页面加入inactive链表
 *
 * 1.**isolate_lru_folios()**，从LRU active链表隔离出指定数量的页面；
 * 2.逐个遍历folio
	- 如果是unevictable页面，则直接加入LRU unevictable链表；
	- 如果是引用计数为0且可执行的文件页面，则还是返回active链表；
	- 将下面类型的页面加入inactive链表，需要清除PG_active
		- 匿名页;
		- 不可执行的文件页面;
		- 可执行但没有引用计数为0的文件页面;
 * 将l_active和l_inactive直接加入对应的LRU链表中
 */
static void shrink_active_list(unsigned long nr_to_scan,
			       struct lruvec *lruvec,
			       struct scan_control *sc,
			       enum lru_list lru)
{
	unsigned long nr_taken;
	unsigned long nr_scanned;
	unsigned long vm_flags;
	LIST_HEAD(l_hold);	/* The folios which were snipped off */
	LIST_HEAD(l_active);
	LIST_HEAD(l_inactive);
	unsigned nr_deactivate, nr_activate;
	unsigned nr_rotated = 0;
	bool file = is_file_lru(lru);
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);

	lru_add_drain();

	spin_lock_irq(&lruvec->lru_lock);

	/*
	 * 从指定的LRU链表尾部中隔离出指定数量的页面, 放到l_hold中
	 *
	 * nr_to_scan: 计划扫描的页面
	 * nr_taken: 实际隔离出的页面数量
	 * nr_scanned: 实际扫描的总页面数量（包含不合格的页面）
	 *
	 * 不参与隔离的几种页面
	 * 1.所在zone编号高于本次回收规定的最高zone的页面；
	 * 2.没有PG_lru标志的页面（并发隔离）；
	 * 3.如果sc不允许接触映射，但仍有映射的页面；
	 * 4.无法增加引用计数的页面（说明正在被释放）；
	 */
	nr_taken = isolate_lru_folios(nr_to_scan, lruvec, &l_hold,
				     &nr_scanned, sc, lru);

	__mod_node_page_state(pgdat, NR_ISOLATED_ANON + file, nr_taken);

	if (!cgroup_reclaim(sc))
		__count_vm_events(PGREFILL, nr_scanned);
	__count_memcg_events(lruvec_memcg(lruvec), PGREFILL, nr_scanned);

	spin_unlock_irq(&lruvec->lru_lock);

	while (!list_empty(&l_hold)) {
		struct folio *folio;

		cond_resched();
		/* 从l_hold尾部取出一个folio */
		folio = lru_to_folio(&l_hold);
		/* 将取出的folio从l_hold链表删除 */
		list_del(&folio->lru);

		/* 将unevictable的页面放回对应的LRU链表 */
		if (unlikely(!folio_evictable(folio))) {
			folio_putback_lru(folio);
			continue;
		}

		if (unlikely(buffer_heads_over_limit)) {
			if (folio_needs_release(folio) &&
			    folio_trylock(folio)) {
				filemap_release_folio(folio, 0);
				folio_unlock(folio);
			}
		}

		/* Referenced or rmap lock contention: rotate */
		/*
		 * 如果是引用计数不为0、且可执行的文件页面，则加入l_active链表
		 */
		if (folio_referenced(folio, 0, sc->target_mem_cgroup,
				     &vm_flags) != 0) {
			/*
			 * Identify referenced, file-backed active folios and
			 * give them one more trip around the active list. So
			 * that executable code get better chances to stay in
			 * memory under moderate memory pressure.  Anon folios
			 * are not likely to be evicted by use-once streaming
			 * IO, plus JVM can create lots of anon VM_EXEC folios,
			 * so we ignore them here.
			 */
			if ((vm_flags & VM_EXEC) && folio_is_file_lru(folio)) {
				nr_rotated += folio_nr_pages(folio);
				list_add(&folio->lru, &l_active);
				continue;
			}
		}

		/*
		 * 将下面类型的页面加入inactive链表:
		 *
		 * 1.匿名页;
		 * 2.不可执行的文件页面;
		 * 3.可执行但没有引用计数为0的文件页面;
		 */
		/* 清除PG_active标志 */
		folio_clear_active(folio);	/* we are de-activating */
		/* 标记该folio正在被使用(工作集) */
		folio_set_workingset(folio);
		/* 加入l_inactive链表 */
		list_add(&folio->lru, &l_inactive);
	}

	/*
	 * Move folios back to the lru list.
	 */
	spin_lock_irq(&lruvec->lru_lock);

	/*
	 * 将l_active和l_inactive直接加入对应的LRU链表中
	 */
	nr_activate = move_folios_to_lru(lruvec, &l_active);
	nr_deactivate = move_folios_to_lru(lruvec, &l_inactive);

	__count_vm_events(PGDEACTIVATE, nr_deactivate);
	__count_memcg_events(lruvec_memcg(lruvec), PGDEACTIVATE, nr_deactivate);

	__mod_node_page_state(pgdat, NR_ISOLATED_ANON + file, -nr_taken);
	spin_unlock_irq(&lruvec->lru_lock);

	if (nr_rotated)
		lru_note_cost(lruvec, file, 0, nr_rotated);
	trace_mm_vmscan_lru_shrink_active(pgdat->node_id, nr_taken, nr_activate,
			nr_deactivate, nr_rotated, sc->priority, file);
}

static unsigned int reclaim_folio_list(struct list_head *folio_list,
				      struct pglist_data *pgdat)
{
	struct reclaim_stat dummy_stat;
	unsigned int nr_reclaimed;
	struct folio *folio;
	struct scan_control sc = {
		.gfp_mask = GFP_KERNEL,
		.may_writepage = 1,
		.may_unmap = 1,
		.may_swap = 1,
		.no_demotion = 1,
	};

	nr_reclaimed = shrink_folio_list(folio_list, pgdat, &sc, &dummy_stat, true);
	while (!list_empty(folio_list)) {
		folio = lru_to_folio(folio_list);
		list_del(&folio->lru);
		folio_putback_lru(folio);
	}

	return nr_reclaimed;
}

unsigned long reclaim_pages(struct list_head *folio_list)
{
	int nid;
	unsigned int nr_reclaimed = 0;
	LIST_HEAD(node_folio_list);
	unsigned int noreclaim_flag;

	if (list_empty(folio_list))
		return nr_reclaimed;

	noreclaim_flag = memalloc_noreclaim_save();

	nid = folio_nid(lru_to_folio(folio_list));
	do {
		struct folio *folio = lru_to_folio(folio_list);

		if (nid == folio_nid(folio)) {
			folio_clear_active(folio);
			list_move(&folio->lru, &node_folio_list);
			continue;
		}

		nr_reclaimed += reclaim_folio_list(&node_folio_list, NODE_DATA(nid));
		nid = folio_nid(lru_to_folio(folio_list));
	} while (!list_empty(folio_list));

	nr_reclaimed += reclaim_folio_list(&node_folio_list, NODE_DATA(nid));

	memalloc_noreclaim_restore(noreclaim_flag);

	return nr_reclaimed;
}

/*
 * 老化/回收LRU链表
 *	- **shrink_active_list**, 老化LRU active链表；
 *	- **shrink_inactive_list**, 回收LRU inactive链表页面内存；
 */
static unsigned long shrink_list(enum lru_list lru, unsigned long nr_to_scan,
				 struct lruvec *lruvec, struct scan_control *sc)
{
	/* 如果是回收active链表 */
	if (is_active_lru(lru)) {
		/*
		 * 检查是否允许回收活跃页面:
		 * sc->may_deactivate 位图标识哪些类型的活跃链表可以回收
		 * 1 << is_file_lru(lru) - 根据LRU类型(文件/匿名)生成对应的位掩码
		 */
		if (sc->may_deactivate & (1 << is_file_lru(lru)))
			/* 收缩指定的LRU active链表 */
			shrink_active_list(nr_to_scan, lruvec, sc, lru);
		else
			/* 跳过活跃链表回收(可能因为系统压力不够大) */
			sc->skipped_deactivate = 1;
		return 0;
	}

	/* 回收LRU inactive 链表 */
	return shrink_inactive_list(nr_to_scan, lruvec, sc, lru);
}

/*
 * The inactive anon list should be small enough that the VM never has
 * to do too much work.
 *
 * The inactive file list should be small enough to leave most memory
 * to the established workingset on the scan-resistant active list,
 * but large enough to avoid thrashing the aggregate readahead window.
 *
 * Both inactive lists should also be large enough that each inactive
 * folio has a chance to be referenced again before it is reclaimed.
 *
 * If that fails and refaulting is observed, the inactive list grows.
 *
 * The inactive_ratio is the target ratio of ACTIVE to INACTIVE folios
 * on this LRU, maintained by the pageout code. An inactive_ratio
 * of 3 means 3:1 or 25% of the folios are kept on the inactive list.
 *
 * total     target    max
 * memory    ratio     inactive
 * -------------------------------------
 *   10MB       1         5MB
 *  100MB       1        50MB
 *    1GB       3       250MB
 *   10GB      10       0.9GB
 *  100GB      31         3GB
 *    1TB     101        10GB
 *   10TB     320        32GB
 */
static bool inactive_is_low(struct lruvec *lruvec, enum lru_list inactive_lru)
{
	enum lru_list active_lru = inactive_lru + LRU_ACTIVE;
	unsigned long inactive, active;
	unsigned long inactive_ratio;
	unsigned long gb;

	inactive = lruvec_page_state(lruvec, NR_LRU_BASE + inactive_lru);
	active = lruvec_page_state(lruvec, NR_LRU_BASE + active_lru);

	gb = (inactive + active) >> (30 - PAGE_SHIFT);
	if (gb)
		inactive_ratio = int_sqrt(10 * gb);
	else
		inactive_ratio = 1;

	return inactive * inactive_ratio < active;
}

enum scan_balance {
	SCAN_EQUAL,	/* 平等扫描匿名页和文件页*/
	SCAN_FRACT,	/* 根据成本(cost)和swappiness决定匿名页和文件页的扫描比例 */
	SCAN_ANON,	/* 只扫描匿名页 */
	SCAN_FILE,	/* 只扫描文件页 */
};

/*
 * 更新sc参数
 *	- 使能了MGLRU，则跳过；
 *	- 更新anon_cost和file_cost；
 *	- 更新may_deactivate，根据force_deactivate、refault和inactive_is_low来设置；
 *	- 更新cache_trim_mode，根据不活跃文件页面是否足够多来设置；
 *	- 更新file_is_tiny, 根据文件页面是否极少、不活跃匿名页面是否足够多来设置；
 */
static void prepare_scan_control(pg_data_t *pgdat, struct scan_control *sc)
{
	unsigned long file;
	struct lruvec *target_lruvec;

	/* MGLRU有自己的平衡策略 */
	if (lru_gen_enabled())
		return;

	target_lruvec = mem_cgroup_lruvec(sc->target_mem_cgroup, pgdat);

	/*
	 * Flush the memory cgroup stats in rate-limited way as we don't need
	 * most accurate stats here. We may switch to regular stats flushing
	 * in the future once it is cheap enough.
	 */
	/*
	 * 以限速的方式刷新memcg的统计信息
	 */
	mem_cgroup_flush_stats_ratelimited(sc->target_mem_cgroup);

	/*
	 * Determine the scan balance between anon and file LRUs.
	 */
        /*
         * 确定anon和file LRU之间的扫描平衡：
         * - 从lruvec读取anon_cost和file_cost
         * - 这些成本反映了anon和file页面的回收难度
         */
	spin_lock_irq(&target_lruvec->lru_lock);
	sc->anon_cost = target_lruvec->anon_cost;
	sc->file_cost = target_lruvec->file_cost;
	spin_unlock_irq(&target_lruvec->lru_lock);

	/*
	 * Target desirable inactive:active list ratios for the anon
	 * and file LRU lists.
	 */
	/*
	 * force_deactivate为false，说明当前系统内存压力还可以,
	 * 根据实际情况决定老化active链表，也就是调整inactive\active的比例。
	 *	- 如果发生了refault，或者inactive_is_low发现inactive页面过少，
	 *	  则设置对应的may_deactivate标记，表示需要老化active链表，
	 *	  anon list设置DEACTIVATE_ANON, file list设置DEACTIVATE_FILE。
	 *
	 * force_deactivate为true, 说明当前系统内存压力较大，
	 *	- 同时老化anon 和file active list。
	 */
	if (!sc->force_deactivate) {
		unsigned long refaults;

		/*
		 * When refaults are being observed, it means a new
		 * workingset is being established(建立). Deactivate to get
		 * rid of any stale(过时的、不新鲜的) active pages quickly.
		 */
		/*
		 * 如果发生了refault, 说明有新的工作集在产生，
		 * 则需要尽快老化inactive链表，来驱逐比较老的active页面
		 */
		refaults = lruvec_page_state(target_lruvec,
				WORKINGSET_ACTIVATE_ANON);
		if (refaults != target_lruvec->refaults[WORKINGSET_ANON] ||
			inactive_is_low(target_lruvec, LRU_INACTIVE_ANON))
			sc->may_deactivate |= DEACTIVATE_ANON;
		else
			sc->may_deactivate &= ~DEACTIVATE_ANON;

		refaults = lruvec_page_state(target_lruvec,
				WORKINGSET_ACTIVATE_FILE);
		if (refaults != target_lruvec->refaults[WORKINGSET_FILE] ||
		    inactive_is_low(target_lruvec, LRU_INACTIVE_FILE))
			sc->may_deactivate |= DEACTIVATE_FILE;
		else
			sc->may_deactivate &= ~DEACTIVATE_FILE;
	} else
		sc->may_deactivate = DEACTIVATE_ANON | DEACTIVATE_FILE;

	/*
	 * If we have plenty of inactive file pages that aren't
	 * thrashing, try to reclaim those first before touching
	 * anonymous pages.
	 */
	/*
	 * **thrashing, 内存颠簸**
	 *	- 系统在频繁swapin和swapout;
	 *	- 系统内存严重不足时，系统在内存和磁盘之间来回交换页面，导致cpu利用率急剧下降；
	 *	- 可以用vmstat 1，查看系统swapin和swapout的实时增长来判断；
	 *	- 可以调整swappiness, 减少匿名页的回收，增加文件页面的回收；
	 */
	/*
	 * 如果有大量inactive文件页面，且是非颠簸的，
	 * 则设置cache_trim_mode为1，优先考虑回收文件页面，
	 *
	 * 反之，则设置cache_trim_mode为0，表示不优先回收文件页面,
	 * 但是不表示优先回收匿名页面，需要结合其它条件。
	 *
	 *  - file >> sc->priority为true, 表示inactive文件页面足够多;
	 *  - !(sc->may_deactivate & DEACTIVATE_FILE)为true，表示不需要老化acitve file list，
	 *    也说明inactive文件页面够多
	 *  - !sc->no_cache_trim_mode为true，表示回收文件页面没失败过，
	 *    则可以设置cache_trim_mode为1，否则就没必要了。
	 */
	file = lruvec_page_state(target_lruvec, NR_INACTIVE_FILE);
	if (file >> sc->priority && !(sc->may_deactivate & DEACTIVATE_FILE) &&
	    !sc->no_cache_trim_mode)
		sc->cache_trim_mode = 1;
	else
		sc->cache_trim_mode = 0;

	/*
	 * Prevent the reclaimer from falling into the cache trap: as
	 * cache pages start out inactive, every cache fault will tip
	 * the scan balance towards the file LRU.  And as the file LRU
	 * shrinks, so does the window for rotation from references.
	 * This means we have a runaway feedback loop where a tiny
	 * thrashing file LRU becomes infinitely more attractive than
	 * anon pages.  Try to detect this based on file LRU size.
	 */
	/*
	 * !cgroup_reclaim(sc)为true，表示全局回收，判断是否设置file_is_tiny
	 */
	if (!cgroup_reclaim(sc)) {
		unsigned long total_high_wmark = 0;
		unsigned long free, anon;
		int z;

		free = sum_zone_node_page_state(pgdat->node_id, NR_FREE_PAGES);
		file = node_page_state(pgdat, NR_ACTIVE_FILE) +
			   node_page_state(pgdat, NR_INACTIVE_FILE);

		for (z = 0; z < MAX_NR_ZONES; z++) {
			struct zone *zone = &pgdat->node_zones[z];

			if (!managed_zone(zone))
				continue;

			total_high_wmark += high_wmark_pages(zone);
		}

		/*
		 * Consider anon: if that's low too, this isn't a
		 * runaway file reclaim problem, but rather just
		 * extreme pressure. Reclaim as per usual then.
		 */
		anon = node_page_state(pgdat, NR_INACTIVE_ANON);

		/*
		 * 文件页面极少、inactive匿名页面足够多，则设置file_is_tiny为true
		 * 后续在get_scan_count中会强制只扫描匿名页面
		 *
		 * file + free <= total_high_wmark为true
		 *	- 文件页 + 空闲页（类似于系统可用内存）小于high水位, 说明此时系统内存资源并不充分；
		 * !(sc->may_deactivate & DEACTIVATE_ANON) 为true
		 *	- sc->may_deactivate &= ~DEACTIVATE_ANON, 说明此时inactive anon page是充足的, 无需从active活跃状态转换到inactive非活跃状态;
		 *	- 最终要表达的含义就是：非活动匿名页是充足的；
		 * anon >> priority为true
		 *	- 说明此时系统anon inactive 匿名页非活动页面是充足的；
		 *
		 * 在这三个条件同时成立的情况时：证明此时系统内存资源不足，文件页是稀少的，
		 * 非活动状态匿名页是充足的，可优先回收非活动状态匿名页，sc->file_is_tiny = true。
		 */
		sc->file_is_tiny =
			file + free <= total_high_wmark &&
			!(sc->may_deactivate & DEACTIVATE_ANON) &&
			anon >> sc->priority;
	}
}

/*
 * Determine how aggressively the anon and file LRU lists should be
 * scanned.
 *
 * nr[0] = anon inactive folios to scan; nr[1] = anon active folios to scan
 * nr[2] = file inactive folios to scan; nr[3] = file active folios to scan
 */
/*
 * 确定每个LRU链表的页面扫描数量，保存到nr数组中
 *	- nr[0] = anon inactive 要扫描的页数
 *	- nr[1] = anon active   要扫描的页数（更多是用于老化/降级）
 *	- nr[2] = file inactive 要扫描的页数
 *	- nr[3] = file active   要扫描的页数（更多是用于老化/降级）
 *
 * 1.确定扫描策略
 *	- SCAN_FILE, 只扫描文件页
 *		+ 不支持swap或者匿名页不可回收；
 *		+ cgroup局部回收，且swappiness为0, 只扫描文件页面;
 *		+ 设置了cache_trim_mode;
 *	- SCAN_ANON, 只扫描匿名页页
 *		+ 设置了file_is_tiny;
 *	- SCAN_EQUAL，等量扫描匿名页和文件页
 *		+ sc->priority为0, 且swappiness不为0;
 *	- SCAN_FRACT, 根据swappiness和cost决定扫描数量（默认策略）
 * 2.根据swappiness和cost计算匿名页和文件页的扫描比例；
 * 3.遍历所有可回收的LRU链表，计算每个链表的具体扫描数量, 最后根据扫描策略确认扫描数量；
 *	- 如果memcg设置了内存保护，则根据low和min， 来计算scan;
 *	- 如果没有设置内存保护，则scan = lruvec_size;
 *	- scan根据sc->priority左移调整；
 *	-根据扫描策略最后确认扫描数量：
 *		+ SCAN_EQUAL, 直接用上面计算好的scan;
 *		+ SCAN_FILE, 用分数制的方式再次计算scan;
 *		+ SCAN_FILE & SCAN_ANON, 针对不扫描的类型，scan赋值0，扫描的类型还是用上面计算好的scan；
 * 4.返回保存扫描数量的数组；
 */
static void get_scan_count(struct lruvec *lruvec, struct scan_control *sc,
			   unsigned long *nr)
{
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);
	struct mem_cgroup *memcg = lruvec_memcg(lruvec);
	unsigned long anon_cost, file_cost, total_cost;
	int swappiness = sc_swappiness(sc, memcg);
	u64 fraction[ANON_AND_FILE];
	u64 denominator = 0;	/* gcc */
	enum scan_balance scan_balance;
	unsigned long ap, fp;
	enum lru_list lru;
	bool balance_anon_file_reclaim = false;

	/* If we have no swap space, do not bother scanning anon folios. */
	/* 1.如果不支持swap或者匿名页不可回收, 则只扫描文件页面 */
	if (!sc->may_swap || !can_reclaim_anon_pages(memcg, pgdat->node_id, sc)) {
		scan_balance = SCAN_FILE;
		goto out;
	}

	/*
	 * Global reclaim will swap to prevent OOM even with no
	 * swappiness, but memcg users want to use this knob to
	 * disable swapping for individual groups completely when
	 * using the memory controller's swap limit feature would be
	 * too expensive.
	 */
	/* 2.如果是cgroup局部回收，且swappiness为0, 只扫描文件页面 */
	if (cgroup_reclaim(sc) && !swappiness) {
		scan_balance = SCAN_FILE;
		goto out;
	}

	/*
	 * Do not apply any pressure balancing cleverness when the
	 * system is close to OOM, scan both anon and file equally
	 * (unless the swappiness setting disagrees with swapping).
	 */
	/*
	 * 3.sc->priority为0表示达到最大扫描力度了，说明系统内存很紧张了，
	 *   这时需要等量扫描匿名页和文件页，尽快释放内存。
	 *   除非当前memcg不支持swap
	 *
	 * 扫描优先级达到0，说明前面一直没回收到内存、内存相当紧张了，开始回收匿名页
	 */
	if (!sc->priority && swappiness) {
		scan_balance = SCAN_EQUAL;
		goto out;
	}

	/*
	 * If the system is almost out of file pages, force-scan anon.
	 */
	/*
	 * 4.如果设置了file_is_tiny，说明几乎没有文件页面，有足够多的inactive匿名页面，
	 *   就只扫描匿名页面。
	 */
	if (sc->file_is_tiny) {
		scan_balance = SCAN_ANON;
		goto out;
	}

	trace_android_rvh_set_balance_anon_file_reclaim(&balance_anon_file_reclaim);

	/*
	 * If there is enough inactive page cache, we do not reclaim
	 * anything from the anonymous working right now. But when balancing
	 * anon and page cache files for reclaim, allow swapping of anon pages
	 * even if there are a number of inactive file cache pages.
	 */
	/*
	 * 5.如果设置了cache_trim_mode, 表示有大量的inactive文件页面，
	 *   则只扫描文件页面
	 */
	if (!balance_anon_file_reclaim && sc->cache_trim_mode) {
		scan_balance = SCAN_FILE;
		goto out;
	}

	/* 6.根据成本(cost)和swappiness决定匿名页和文件页的扫描比例 */
	scan_balance = SCAN_FRACT;
	/*
	 * Calculate the pressure balance between anon and file pages.
	 *
	 * The amount of pressure we put on each LRU is inversely
	 * proportional to the cost of reclaiming each list, as
	 * determined by the share of pages that are refaulting, times
	 * the relative IO cost of bringing back a swapped out
	 * anonymous page vs reloading a filesystem page (swappiness).
	 *
	 * Although we limit that influence to ensure no list gets
	 * left behind completely: at least a third of the pressure is
	 * applied, before swappiness.
	 *
	 * With swappiness at 100, anon and file have equal IO cost.
	 */
	/*
         * 计算匿名页和文件页之间的压力平衡。
         *
         * 我们对每个 lru 施加的压力与回收每个列表的成本成反比，
         * 由发生 refault 的页面份额乘以换出匿名页与重新加载文件系统页面
         * 的相对 io 成本（swappiness）决定。
         *
         * 尽管我们限制了这种影响以确保没有列表完全落后：
         * 在考虑 swappiness 之前，至少施加三分之一的压力。
         *
         * 当 swappiness 为 100 时，匿名和文件页具有相等的 io 成本。
         */
	/*
         * - 基于近期的refault驱动的成本估计（sc->anon_cost/file_cost）
         *   以及swappiness（匿名交换vs文件重载的I/O代价）分配压力；
         * - 成本越高，施压越小；成本越低，施压越大；
         * - 对任一类至少保留约1/3的压力，避免“被完全抛弃”的极端情况。
	 */
	total_cost = sc->anon_cost + sc->file_cost;	/* 总回收成本 */
	anon_cost = total_cost + sc->anon_cost;		/* 匿名页成本（加权） */
	file_cost = total_cost + sc->file_cost;		/* 文件页成本（加权） */
	total_cost = anon_cost + file_cost;		/* 新的总成本 */

	/* 计算匿名页的扫描压力 */
	ap = swappiness * (total_cost + 1);
	ap /= anon_cost + 1;

	/*
	 * 计算文件页的扫描压力
	 *
	 * swappiness越大，越倾向于扫描/回收文件页
	 * 当swappiness等于100时，匿名页:文件页趋近1:1
	 */
	fp = (MAX_SWAPPINESS - swappiness) * (total_cost + 1);
	fp /= file_cost + 1;

	fraction[0] = ap;	/* 匿名页比例 */
	fraction[1] = fp;	/* 文件页比例 */
	denominator = ap + fp;	/* 总比例 */
out:
        /* 遍历所有可回收的 LRU 链表，计算每个链表的具体扫描数量 */
	for_each_evictable_lru(lru) {
		bool file = is_file_lru(lru);
		unsigned long lruvec_size;
		unsigned long low, min;
		unsigned long scan;

		/* 获取该lru链表在目标zone范围内的页面总数 */
		lruvec_size = lruvec_lru_size(lruvec, lru, sc->reclaim_idx);
		mem_cgroup_protection(sc->target_mem_cgroup, memcg,
				      &min, &low);

		/* 如果memcg有内存保护设置，则根据内存保护设置来调整扫描量 */
		if (min || low) {
			/*
			 * Scale a cgroup's reclaim pressure by proportioning
			 * its current usage to its memory.low or memory.min
			 * setting.
			 *
			 * This is important, as otherwise scanning aggression
			 * becomes extremely binary -- from nothing as we
			 * approach the memory protection threshold, to totally
			 * nominal as we exceed it.  This results in requiring
			 * setting extremely liberal protection thresholds. It
			 * also means we simply get no protection at all if we
			 * set it too low, which is not ideal.
			 *
			 * If there is any protection in place, we reduce scan
			 * pressure by how much of the total memory used is
			 * within protection thresholds.
			 *
			 * There is one special case: in the first reclaim pass,
			 * we skip over all groups that are within their low
			 * protection. If that fails to reclaim enough pages to
			 * satisfy the reclaim goal, we come back and override
			 * the best-effort low protection. However, we still
			 * ideally want to honor how well-behaved groups are in
			 * that case instead of simply punishing them all
			 * equally. As such, we reclaim them based on how much
			 * memory they are using, reducing the scan pressure
			 * again by how much of the total memory used is under
			 * hard protection.
			 */
                        /*
                         * 按保护阈值缩放扫描压力：
                         * - 避免“刚过阈值就全压/未过就全不压”的二元行为；
                         * - 第一轮若处于low保护内可先跳过（通过sc->memcg_low_skipped记录），
                         *   若不足目标，再回来按使用量与保护的关系温和施压；
                         * - 计算：按(使用量-保护量)/使用量的比例缩小扫描量，并保底SWAP_CLUSTER_MAX，
                         *   以便维持回收前进、避免priority过快下降。
                         */
			/* 获取memcg当前的内存使用大小 */
			unsigned long cgroup_size = mem_cgroup_size(memcg);
			unsigned long protection;

			/* memory.low scaling, make sure we retry before OOM */
			/*
			 * 如果没有设置memcg_low_reclaim(不保护low)，且low > min，
			 * 则保护内存量为low
			 * 同时设置memcg_low_skipped(保护low)
			 */
			if (!sc->memcg_low_reclaim && low > min) {
				protection = low;
				sc->memcg_low_skipped = 1;
			/*
			 * 如果设置了memcg_low_reclaim，或者low <= min，
			 * 则保护内存量为min
			 */
			} else {
				protection = min;
			}

			/* Avoid TOCTOU with earlier protection check */
			cgroup_size = max(cgroup_size, protection);

			/*
			 * 关键: 根据保护比例调整扫描数量
			 */
			scan = lruvec_size - lruvec_size * protection /
				(cgroup_size + 1);

			/*
			 * Minimally target SWAP_CLUSTER_MAX pages to keep
			 * reclaim moving forwards, avoiding decrementing
			 * sc->priority further than desirable.
			 */
			/*
			 * 确保scan最小值为SWAP_CLUSTER_MAX
			 * 确保回收能正常进行，避免扫描页面过少而做无用功
			 */
			scan = max(scan, SWAP_CLUSTER_MAX);
		} else {
                        /* 没有保护限制，扫描所有页面 */
			scan = lruvec_size;
		}

                /* 根据优先级调整扫描数量（优先级越高，扫描越少） */
		scan >>= sc->priority;

		/*
		 * If the cgroup's already been deleted, make sure to
		 * scrape out (剔除) the remaining cache.
		 */
                /* 如果 cgroup 已被删除，确保清理剩余的缓存*/
                /* 离线memcg的擦尾：给一点扫描量刮净残留cache */
		if (!scan && !mem_cgroup_online(memcg))
			scan = min(lruvec_size, SWAP_CLUSTER_MAX);

                /* 根据选择的平衡策略进一步调整扫描数量 */
		switch (scan_balance) {
		case SCAN_EQUAL:
			/* Scan lists relative to size */
                        /* 等量：按规模相对均匀扫描（已在上面通过scan确定） */
			break;
		case SCAN_FRACT:
			/*
			 * Scan types proportional to swappiness and
			 * their relative recent reclaim efficiency.
			 * Make sure we don't miss the last page on
			 * the offlined memory cgroups because of a
			 * round-off error.
			 */
                        /*
                         * 分数制：按fraction[file?/anon?]/denominator比例调整scan，
                         * 离线memcg用向上取整避免漏最后一页。
                         */
			scan = mem_cgroup_online(memcg) ?
			       div64_u64(scan * fraction[file], denominator) :
			       DIV64_U64_ROUND_UP(scan * fraction[file],
						  denominator);
			break;
		case SCAN_FILE:
		case SCAN_ANON:
			/* Scan one type exclusively */
			/* 只扫描一种类型
			 * 不是目标类型则不扫描，设置扫描量为0
			 */
			if ((scan_balance == SCAN_FILE) != file)
				scan = 0;
			break;
		default:
			/* Look ma, no brain */
			BUG();
		}

		/* 按照LRU链表类型保存扫描量 */
		nr[lru] = scan;
	}
}

/*
 * Anonymous LRU management is a waste if there is
 * ultimately no way to reclaim the memory.
 */
static bool can_age_anon_pages(struct pglist_data *pgdat,
			       struct scan_control *sc)
{
	/* Aging the anon LRU is valuable if swap is present: */
	if (total_swap_pages > 0)
		return true;

	/* Also valuable if anon pages can be demoted: */
	return can_demote(pgdat->node_id, sc);
}

#ifdef CONFIG_LRU_GEN

#ifdef CONFIG_LRU_GEN_ENABLED
DEFINE_STATIC_KEY_ARRAY_TRUE(lru_gen_caps, NR_LRU_GEN_CAPS);
#define get_cap(cap)	static_branch_likely(&lru_gen_caps[cap])
#else
DEFINE_STATIC_KEY_ARRAY_FALSE(lru_gen_caps, NR_LRU_GEN_CAPS);
#define get_cap(cap)	static_branch_unlikely(&lru_gen_caps[cap])
#endif

static bool should_walk_mmu(void)
{
	return arch_has_hw_pte_young() && get_cap(LRU_GEN_MM_WALK);
}

static bool should_clear_pmd_young(void)
{
	return arch_has_hw_nonleaf_pmd_young() && get_cap(LRU_GEN_NONLEAF_YOUNG);
}

/******************************************************************************
 *                          shorthand helpers
 ******************************************************************************/

#define LRU_REFS_FLAGS	(BIT(PG_referenced) | BIT(PG_workingset))

#define DEFINE_MAX_SEQ(lruvec)						\
	unsigned long max_seq = READ_ONCE((lruvec)->lrugen.max_seq)

#define DEFINE_MIN_SEQ(lruvec)						\
	unsigned long min_seq[ANON_AND_FILE] = {			\
		READ_ONCE((lruvec)->lrugen.min_seq[LRU_GEN_ANON]),	\
		READ_ONCE((lruvec)->lrugen.min_seq[LRU_GEN_FILE]),	\
	}

#define for_each_gen_type_zone(gen, type, zone)				\
	for ((gen) = 0; (gen) < MAX_NR_GENS; (gen)++)			\
		for ((type) = 0; (type) < ANON_AND_FILE; (type)++)	\
			for ((zone) = 0; (zone) < MAX_NR_ZONES; (zone)++)

#define get_memcg_gen(seq)	((seq) % MEMCG_NR_GENS)
#define get_memcg_bin(bin)	((bin) % MEMCG_NR_BINS)

/*
 * 获取memcg在特定NUMA node上的lruvec
 */
static struct lruvec *get_lruvec(struct mem_cgroup *memcg, int nid)
{
	struct pglist_data *pgdat = NODE_DATA(nid);

#ifdef CONFIG_MEMCG
	if (memcg) {
		struct lruvec *lruvec = &memcg->nodeinfo[nid]->lruvec;

		/* see the comment in mem_cgroup_lruvec() */
		if (!lruvec->pgdat)
			lruvec->pgdat = pgdat;

		return lruvec;
	}
#endif
	VM_WARN_ON_ONCE(!mem_cgroup_disabled());

	return &pgdat->__lruvec;
}

static int get_swappiness(struct lruvec *lruvec, struct scan_control *sc)
{
	struct mem_cgroup *memcg = lruvec_memcg(lruvec);
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);

	if (!sc->may_swap)
		return 0;

	if (!can_demote(pgdat->node_id, sc) &&
	    mem_cgroup_get_nr_swap_pages(memcg) < MIN_LRU_BATCH)
		return 0;

	return sc_swappiness(sc, memcg);
}

static int get_nr_gens(struct lruvec *lruvec, int type)
{
	return lruvec->lrugen.max_seq - lruvec->lrugen.min_seq[type] + 1;
}

static bool __maybe_unused seq_is_valid(struct lruvec *lruvec)
{
	/* see the comment on lru_gen_folio */
	return get_nr_gens(lruvec, LRU_GEN_FILE) >= MIN_NR_GENS &&
	       get_nr_gens(lruvec, LRU_GEN_FILE) <= get_nr_gens(lruvec, LRU_GEN_ANON) &&
	       get_nr_gens(lruvec, LRU_GEN_ANON) <= MAX_NR_GENS;
}

/******************************************************************************
 *                          Bloom filters
 ******************************************************************************/

/*
 * Bloom filters with m=1<<15, k=2 and the false positive rates of ~1/5 when
 * n=10,000 and ~1/2 when n=20,000, where, conventionally, m is the number of
 * bits in a bitmap, k is the number of hash functions and n is the number of
 * inserted items.
 *
 * Page table walkers use one of the two filters to reduce their search space.
 * To get rid of non-leaf entries that no longer have enough leaf entries, the
 * aging uses the double-buffering technique to flip to the other filter each
 * time it produces a new generation. For non-leaf entries that have enough
 * leaf entries, the aging carries them over to the next generation in
 * walk_pmd_range(); the eviction also report them when walking the rmap
 * in lru_gen_look_around().
 *
 * For future optimizations:
 * 1. It's not necessary to keep both filters all the time. The spare one can be
 *    freed after the RCU grace period and reallocated if needed again.
 * 2. And when reallocating, it's worth scaling its size according to the number
 *    of inserted entries in the other filter, to reduce the memory overhead on
 *    small systems and false positives on large systems.
 * 3. Jenkins' hash function is an alternative to Knuth's.
 */
#define BLOOM_FILTER_SHIFT	15

static inline int filter_gen_from_seq(unsigned long seq)
{
	return seq % NR_BLOOM_FILTERS;
}

static void get_item_key(void *item, int *key)
{
	u32 hash = hash_ptr(item, BLOOM_FILTER_SHIFT * 2);

	BUILD_BUG_ON(BLOOM_FILTER_SHIFT * 2 > BITS_PER_TYPE(u32));

	key[0] = hash & (BIT(BLOOM_FILTER_SHIFT) - 1);
	key[1] = hash >> BLOOM_FILTER_SHIFT;
}

static bool test_bloom_filter(struct lru_gen_mm_state *mm_state, unsigned long seq,
			      void *item)
{
	int key[2];
	unsigned long *filter;
	int gen = filter_gen_from_seq(seq);

	filter = READ_ONCE(mm_state->filters[gen]);
	if (!filter)
		return true;

	get_item_key(item, key);

	return test_bit(key[0], filter) && test_bit(key[1], filter);
}

static void update_bloom_filter(struct lru_gen_mm_state *mm_state, unsigned long seq,
				void *item)
{
	int key[2];
	unsigned long *filter;
	int gen = filter_gen_from_seq(seq);

	filter = READ_ONCE(mm_state->filters[gen]);
	if (!filter)
		return;

	get_item_key(item, key);

	if (!test_bit(key[0], filter))
		set_bit(key[0], filter);
	if (!test_bit(key[1], filter))
		set_bit(key[1], filter);
}

static void reset_bloom_filter(struct lru_gen_mm_state *mm_state, unsigned long seq)
{
	unsigned long *filter;
	int gen = filter_gen_from_seq(seq);

	filter = mm_state->filters[gen];
	if (filter) {
		bitmap_clear(filter, 0, BIT(BLOOM_FILTER_SHIFT));
		return;
	}

	filter = bitmap_zalloc(BIT(BLOOM_FILTER_SHIFT),
			       __GFP_HIGH | __GFP_NOMEMALLOC | __GFP_NOWARN);
	WRITE_ONCE(mm_state->filters[gen], filter);
}

/******************************************************************************
 *                          mm_struct list
 ******************************************************************************/

#ifdef CONFIG_LRU_GEN_WALKS_MMU

static struct lru_gen_mm_list *get_mm_list(struct mem_cgroup *memcg)
{
	static struct lru_gen_mm_list mm_list = {
		.fifo = LIST_HEAD_INIT(mm_list.fifo),
		.lock = __SPIN_LOCK_UNLOCKED(mm_list.lock),
	};

#ifdef CONFIG_MEMCG
	if (memcg)
		return &memcg->mm_list;
#endif
	VM_WARN_ON_ONCE(!mem_cgroup_disabled());

	return &mm_list;
}

static struct lru_gen_mm_state *get_mm_state(struct lruvec *lruvec)
{
	return &lruvec->mm_state;
}

static struct mm_struct *get_next_mm(struct lru_gen_mm_walk *walk)
{
	int key;
	struct mm_struct *mm;
	struct pglist_data *pgdat = lruvec_pgdat(walk->lruvec);
	struct lru_gen_mm_state *mm_state = get_mm_state(walk->lruvec);

	mm = list_entry(mm_state->head, struct mm_struct, lru_gen.list);
	key = pgdat->node_id % BITS_PER_TYPE(mm->lru_gen.bitmap);

	/*
	 * 如果bitmap没置位，则跳过该mm
	 *
	 * bitmap 由调度器的 finish_task_switch() 和 exec 路径通过 lru_gen_use_mm() 置位：

		// kernel/sched/core.c:5410
		lru_gen_use_mm(next->mm);   // ← 每次上下文切换到该进程时

		// fs/exec.c:1016
		lru_gen_use_mm(mm);         // ← exec 时

		static inline void lru_gen_use_mm(struct mm_struct *mm)
		{
		WRITE_ONCE(mm->lru_gen.bitmap, -1);  // 所有 bit 全置 1
		}

	 * 结论：
		只有"自上次老化以来曾被调度运行过"的进程，其 mm 才会被扫描（bitmap != 0）；
	 *	从未运行的进程自动跳过。 force_scan=true 时忽略此过滤，强扫所有 mm。
	 */
	if (!walk->force_scan && !test_bit(key, &mm->lru_gen.bitmap))
		return NULL;

	clear_bit(key, &mm->lru_gen.bitmap);

	return mmget_not_zero(mm) ? mm : NULL;
}

void lru_gen_add_mm(struct mm_struct *mm)
{
	int nid;
	struct mem_cgroup *memcg = get_mem_cgroup_from_mm(mm);
	struct lru_gen_mm_list *mm_list = get_mm_list(memcg);

	VM_WARN_ON_ONCE(!list_empty(&mm->lru_gen.list));
#ifdef CONFIG_MEMCG
	VM_WARN_ON_ONCE(mm->lru_gen.memcg);
	mm->lru_gen.memcg = memcg;
#endif
	spin_lock(&mm_list->lock);

	for_each_node_state(nid, N_MEMORY) {
		struct lruvec *lruvec = get_lruvec(memcg, nid);
		struct lru_gen_mm_state *mm_state = get_mm_state(lruvec);

		/* the first addition since the last iteration */
		if (mm_state->tail == &mm_list->fifo)
			mm_state->tail = &mm->lru_gen.list;
	}

	list_add_tail(&mm->lru_gen.list, &mm_list->fifo);

	spin_unlock(&mm_list->lock);
}

void lru_gen_del_mm(struct mm_struct *mm)
{
	int nid;
	struct lru_gen_mm_list *mm_list;
	struct mem_cgroup *memcg = NULL;

	if (list_empty(&mm->lru_gen.list))
		return;

#ifdef CONFIG_MEMCG
	memcg = mm->lru_gen.memcg;
#endif
	mm_list = get_mm_list(memcg);

	spin_lock(&mm_list->lock);

	for_each_node(nid) {
		struct lruvec *lruvec = get_lruvec(memcg, nid);
		struct lru_gen_mm_state *mm_state = get_mm_state(lruvec);

		/* where the current iteration continues after */
		if (mm_state->head == &mm->lru_gen.list)
			mm_state->head = mm_state->head->prev;

		/* where the last iteration ended before */
		if (mm_state->tail == &mm->lru_gen.list)
			mm_state->tail = mm_state->tail->next;
	}

	list_del_init(&mm->lru_gen.list);

	spin_unlock(&mm_list->lock);

#ifdef CONFIG_MEMCG
	mem_cgroup_put(mm->lru_gen.memcg);
	mm->lru_gen.memcg = NULL;
#endif
}

#ifdef CONFIG_MEMCG
void lru_gen_migrate_mm(struct mm_struct *mm)
{
	struct mem_cgroup *memcg;
	struct task_struct *task = rcu_dereference_protected(mm->owner, true);

	VM_WARN_ON_ONCE(task->mm != mm);
	lockdep_assert_held(&task->alloc_lock);

	/* for mm_update_next_owner() */
	if (mem_cgroup_disabled())
		return;

	/* migration can happen before addition */
	if (!mm->lru_gen.memcg)
		return;

	rcu_read_lock();
	memcg = mem_cgroup_from_task(task);
	rcu_read_unlock();
	if (memcg == mm->lru_gen.memcg)
		return;

	VM_WARN_ON_ONCE(list_empty(&mm->lru_gen.list));

	lru_gen_del_mm(mm);
	lru_gen_add_mm(mm);
}
#endif

#else /* !CONFIG_LRU_GEN_WALKS_MMU */

static struct lru_gen_mm_list *get_mm_list(struct mem_cgroup *memcg)
{
	return NULL;
}

static struct lru_gen_mm_state *get_mm_state(struct lruvec *lruvec)
{
	return NULL;
}

static struct mm_struct *get_next_mm(struct lru_gen_mm_walk *walk)
{
	return NULL;
}

#endif

static void reset_mm_stats(struct lru_gen_mm_walk *walk, bool last)
{
	int i;
	int hist;
	struct lruvec *lruvec = walk->lruvec;
	struct lru_gen_mm_state *mm_state = get_mm_state(lruvec);

	lockdep_assert_held(&get_mm_list(lruvec_memcg(lruvec))->lock);

	hist = lru_hist_from_seq(walk->seq);

	for (i = 0; i < NR_MM_STATS; i++) {
		WRITE_ONCE(mm_state->stats[hist][i],
			   mm_state->stats[hist][i] + walk->mm_stats[i]);
		walk->mm_stats[i] = 0;
	}

	if (NR_HIST_GENS > 1 && last) {
		hist = lru_hist_from_seq(walk->seq + 1);

		for (i = 0; i < NR_MM_STATS; i++)
			WRITE_ONCE(mm_state->stats[hist][i], 0);
	}
}

/*
 * iterate_mm_list() 以 mm_list->fifo 为环形链表，使用 mm_state->head 作为游标，每次调用取一个 mm：

   fifo head → mm_A → mm_B → mm_C → mm_D → (回到 fifo head = 结束)
                        ↑
                  mm_state->head（当前游标）

 * 当 mm_state->head 绕回 &mm_list->fifo 时，设 mm_state->seq++（推进轮次），
 * 返回 last=true，告知调用者"本轮所有 mm 均已处理完毕"。
 */
static bool iterate_mm_list(struct lru_gen_mm_walk *walk, struct mm_struct **iter)
{
	bool first = false;
	bool last = false;
	struct mm_struct *mm = NULL;
	struct lruvec *lruvec = walk->lruvec;
	struct mem_cgroup *memcg = lruvec_memcg(lruvec);
	struct lru_gen_mm_list *mm_list = get_mm_list(memcg);
	struct lru_gen_mm_state *mm_state = get_mm_state(lruvec);

	/*
	 * mm_state->seq is incremented after each iteration of mm_list. There
	 * are three interesting cases for this page table walker:
	 * 1. It tries to start a new iteration with a stale max_seq: there is
	 *    nothing left to do.
	 * 2. It started the next iteration: it needs to reset the Bloom filter
	 *    so that a fresh set of PTE tables can be recorded.
	 * 3. It ended the current iteration: it needs to reset the mm stats
	 *    counters and tell its caller to increment max_seq.
	 */
	spin_lock(&mm_list->lock);

	VM_WARN_ON_ONCE(mm_state->seq + 1 < walk->seq);

	if (walk->seq <= mm_state->seq)
		goto done;

	if (!mm_state->head)
		mm_state->head = &mm_list->fifo;

	if (mm_state->head == &mm_list->fifo)
		first = true;

	do {
		mm_state->head = mm_state->head->next;
		if (mm_state->head == &mm_list->fifo) {
			WRITE_ONCE(mm_state->seq, mm_state->seq + 1);
			last = true;
			break;
		}

		/* force scan for those added after the last iteration */
		if (!mm_state->tail || mm_state->tail == mm_state->head) {
			mm_state->tail = mm_state->head->next;
			walk->force_scan = true;
		}
	} while (!(mm = get_next_mm(walk)));
done:
	if (*iter || last)
		reset_mm_stats(walk, last);

	spin_unlock(&mm_list->lock);

	if (mm && first)
		reset_bloom_filter(mm_state, walk->seq + 1);

	if (*iter)
		mmput_async(*iter);

	*iter = mm;

	return last;
}

static bool iterate_mm_list_nowalk(struct lruvec *lruvec, unsigned long seq)
{
	bool success = false;
	struct mem_cgroup *memcg = lruvec_memcg(lruvec);
	struct lru_gen_mm_list *mm_list = get_mm_list(memcg);
	struct lru_gen_mm_state *mm_state = get_mm_state(lruvec);

	spin_lock(&mm_list->lock);

	VM_WARN_ON_ONCE(mm_state->seq + 1 < seq);

	if (seq > mm_state->seq) {
		mm_state->head = NULL;
		mm_state->tail = NULL;
		WRITE_ONCE(mm_state->seq, mm_state->seq + 1);
		success = true;
	}

	spin_unlock(&mm_list->lock);

	return success;
}

/******************************************************************************
 *                          PID controller
 ******************************************************************************/

/*
 * A feedback loop based on Proportional-Integral-Derivative (PID) controller.
 *
 * The P term is refaulted/(evicted+protected) from a tier in the generation
 * currently being evicted; the I term is the exponential moving average of the
 * P term over the generations previously evicted, using the smoothing factor
 * 1/2; the D term isn't supported.
 *
 * The setpoint (SP) is always the first tier of one type; the process variable
 * (PV) is either any tier of the other type or any other tier of the same
 * type.
 *
 * The error is the difference between the SP and the PV; the correction is to
 * turn off protection when SP>PV or turn on protection when SP<PV.
 *
 * For future optimizations:
 * 1. The D term may discount the other two terms over time so that long-lived
 *    generations can resist stale information.
 */
struct ctrl_pos {
	unsigned long refaulted;
	unsigned long total;
	int gain;
};

static void read_ctrl_pos(struct lruvec *lruvec, int type, int tier, int gain,
			  struct ctrl_pos *pos)
{
	struct lru_gen_folio *lrugen = &lruvec->lrugen;
	int hist = lru_hist_from_seq(lrugen->min_seq[type]);

	pos->refaulted = lrugen->avg_refaulted[type][tier] +
			 atomic_long_read(&lrugen->refaulted[hist][type][tier]);
	pos->total = lrugen->avg_total[type][tier] +
		     atomic_long_read(&lrugen->evicted[hist][type][tier]);
	if (tier)
		pos->total += lrugen->protected[hist][type][tier - 1];
	pos->gain = gain;
}

static void reset_ctrl_pos(struct lruvec *lruvec, int type, bool carryover)
{
	int hist, tier;
	struct lru_gen_folio *lrugen = &lruvec->lrugen;
	bool clear = carryover ? NR_HIST_GENS == 1 : NR_HIST_GENS > 1;
	unsigned long seq = carryover ? lrugen->min_seq[type] : lrugen->max_seq + 1;

	lockdep_assert_held(&lruvec->lru_lock);

	if (!carryover && !clear)
		return;

	hist = lru_hist_from_seq(seq);

	for (tier = 0; tier < MAX_NR_TIERS; tier++) {
		if (carryover) {
			unsigned long sum;

			sum = lrugen->avg_refaulted[type][tier] +
			      atomic_long_read(&lrugen->refaulted[hist][type][tier]);
			WRITE_ONCE(lrugen->avg_refaulted[type][tier], sum / 2);

			sum = lrugen->avg_total[type][tier] +
			      atomic_long_read(&lrugen->evicted[hist][type][tier]);
			if (tier)
				sum += lrugen->protected[hist][type][tier - 1];
			WRITE_ONCE(lrugen->avg_total[type][tier], sum / 2);
		}

		if (clear) {
			atomic_long_set(&lrugen->refaulted[hist][type][tier], 0);
			atomic_long_set(&lrugen->evicted[hist][type][tier], 0);
			if (tier)
				WRITE_ONCE(lrugen->protected[hist][type][tier - 1], 0);
		}
	}
}

static bool positive_ctrl_err(struct ctrl_pos *sp, struct ctrl_pos *pv)
{
	/*
	 * Return true if the PV has a limited number of refaults or a lower
	 * refaulted/total than the SP.
	 */
	/*
	 * 下面的第二个条件做了算术转换，等价于：
	 * pv->refault / (pv->total * pv->gain) <= (sp->refault + 1) / ((sp->total + MIN_LRU_BATCH) * sp->gain)
	 *
	 * 用tierN替换pv，也就是当前tier；用tier0替换sp
	 * tierN->refault / (tierN->total * 2) <= (tier0->refault + 1) / ((tier0->total + MIN_LRU_BATCH) * 1)
	 *
	 * 去掉常数：
	 * tierN->refault / (tierN->total) <= (tier0->refault / tier0->total) * 2
	 *
	 * 返回false即是：
	 * tierN->refault / (tierN->total) > (tier0->refault / tier0->total) * 2
	 * 也就是tierN的refault大于tier0 refault率的两倍
	 * 也可以说是tierN的refault率远远大于tier0的refault率
	 */
	return pv->refaulted < MIN_LRU_BATCH ||
	       pv->refaulted * (sp->total + MIN_LRU_BATCH) * sp->gain <=
	       (sp->refaulted + 1) * pv->total * pv->gain;
}

/******************************************************************************
 *                          the aging
 ******************************************************************************/

/* promote pages accessed through page tables */
/*
 * 页面被访问了，调用folio_update_gen提升到最新的gen（max_seq % MAX_NR_GENS）
 * CAS无锁操作，不移动链表，只更新flags的LRU_GEN段
 */
static int folio_update_gen(struct folio *folio, int gen)
{
	unsigned long new_flags, old_flags = READ_ONCE(folio->flags);

	/* 最大gen是3 */
	VM_WARN_ON_ONCE(gen >= MAX_NR_GENS);
	/* 调用者必须持有RCU读锁，应为folio_memcg()需要RCU或者memcg pages lock保护memcg稳定 */
	VM_WARN_ON_ONCE(!rcu_read_lock_held());

	do {
		/* lru_gen_del_folio() has isolated this page? */
		/*
		 * 判断页面是否已经被lru_gen_del_folio隔离
		 * flags快照中的LRU_GEN为0，说明页面不在MGLRU链表中了
		 *
		 * gen保存时会+1，正常挂在LRU链表上的folio LRU_GEN >= 1，
		 * 如果LRU_GEN段为0，说明这个folio不在LRU链表上、被隔离了
		 */
		if (!(old_flags & LRU_GEN_MASK)) {
			/* for shrink_folio_list() */
			/*
			 * 隔离状态说明 folio 正在被 shrink_folio_list() 处理
			 * 此时无法更新代号，但可以设置 PG_referenced 作为"曾被访问"的标记
			 * shrink_folio_list() 会检查这个 bit 决定是否回收，相关代码：
			 *
			 * if (lru_gen_enabled() && !ignore_references &&
			 *	folio_mapped(folio) && folio_test_referenced(folio))
			 *		goto keep_locked;   // ← 这个 folio 不被驱逐，放回 LRU
			 *
			 * folio_update_gen本身的目的是将页面提升到最新的gen，如果folio被隔离了，能做的最多就是设置PG_referenced，留下一个"访问记录"
			 * 让 shrink_folio_list 知道"这个页刚被访问过，别驱逐它"，然后通过 move_folios_to_lru 将其重新放回MGLRU，再等下次被 aging 路径扫到时完成真正的 gen 更新。
			 */
			new_flags = old_flags | BIT(PG_referenced);
			/*
			 * question:
			 * 这里是continue，是不是设置PG_referenced后，要等到shrink_folio_list重新将页面放入MGLRU链表才能往下走？
			 * 不是啊，这里continue后会跳到while中的CAS写入，分两种情况：
			 *	1.如果flags没被改动，则将new_flags更新到folio->flags后退出
			 *	2.如果flags被改动了，则更新old_flags，在下一次或者后面某一次，folio->flags会和old_flags一样，最后成功将new_flags写入并退出
			 */
			continue;
		}

		/*
		 * 清除旧的代号、引用计数、引用相关 flags
		 *	LRU_GEN_MASK:  旧代号字段
		 *	LRU_REFS_MASK: 旧的 LRU_REFS 位域（tier 计数）
		 *	LRU_REFS_FLAGS: PG_referenced | PG_workingset
		 * 全部清零，在新代里从 tier 0 重新开始计数
		 */
		new_flags = old_flags & ~(LRU_GEN_MASK | LRU_REFS_MASK | LRU_REFS_FLAGS);
		/*
		 * 写入 gen+1
		 * 保存的时候 +1，获取的时候-1 (folio_lru_gen), 为了区分出不在MGLRU链表的页面
		 */
		new_flags |= (gen + 1UL) << LRU_GEN_PGOFF;
	/*
	 * CAS：若 folio->flags 未被其他 CPU 修改，则原子写入new_flags
	 * 否则将folio->flags更新到old_flags，并重试
	 */
	} while (!try_cmpxchg(&folio->flags, &old_flags, new_flags));

	/* 返回folio原来的gen (未更新前的) */
	return ((old_flags & LRU_GEN_MASK) >> LRU_GEN_PGOFF) - 1;
}
/*
 * 调用lru_update_gen的所有场景及函数调用链
 *
 * 场景 1：walk_pte_range() - PTE 级别页表扫描
 * 场景 2：walk_pmd_range_locked() - PMD 级别大页扫描
 *	场景 1 & 2都是aging 路径的页表扫描，共用一条调用链
  ★ kswapd 入口
  kswapd()
    └─ balance_pgdat()                            [mm/vmscan.c:8477]
         └─ kswapd_shrink_node()                  [mm/vmscan.c:8360]
              └─ shrink_node()                    [mm/vmscan.c:7229]

  ★ 直接回收入口
  try_to_free_pages()                             [mm/vmscan.c:7989]
    └─ do_try_to_free_pages()                     [mm/vmscan.c:7676]
         └─ shrink_zones()                        [mm/vmscan.c:7581]
              └─ shrink_node()                    [mm/vmscan.c:7229]

  ★ memcg 回收入口
  try_to_free_mem_cgroup_pages()                  [mm/vmscan.c:8077]
    └─ do_try_to_free_pages()
         └─ shrink_zones()
              └─ shrink_node()                    [mm/vmscan.c:7229]

  ★ NUMA 节点回收入口
  node_reclaim()                                  [mm/vmscan.c:9288]
    └─ __node_reclaim()                           [mm/vmscan.c:9235]
         └─ shrink_node()                         [mm/vmscan.c:7229]

                    ↓（共同路径）

  shrink_node()                                   [mm/vmscan.c:7229]
    └─ lru_gen_shrink_node()                      [mm/vmscan.c:6130]
         └─ shrink_many()                         [mm/vmscan.c:6039]
              └─ shrink_one()                     [mm/vmscan.c:6018]
                   └─ try_to_shrink_lruvec()      [mm/vmscan.c:5976]
                        └─ get_nr_to_scan()       [mm/vmscan.c:5933]
                             └─ try_to_inc_max_seq()  [mm/vmscan.c:5028]
                                  └─ walk_mm()    [mm/vmscan.c:4760]
                                       └─ walk_page_range()  [mm/vmscan.c:4789]
                                            └─ walk_pud_range()  [mm/vmscan.c:4739]
                                                 └─ walk_pmd_range()  [mm/vmscan.c:4703]
                                                      ├─ walk_pte_range()       [mm/vmscan.c:4496]
                                                      │    └─ folio_update_gen()  ← 调用点1 [vmscan.c:4548]
                                                      │
                                                      └─ walk_pmd_range_locked() [mm/vmscan.c:4562]
                                                           └─ folio_update_gen()  ← 调用点2 [vmscan.c:4632]
 * 场景1说明：
 *	在页表扫描过程中，遍历 PTE 表项，检测页面的访问位（Accessed bit）。如果页面被访问过（young），将其提升到最新 gen（new_gen = max_seq % MAX_NR_GENS）。
 * 场景2说明：
 *	处理 PMD 级别的大页（THP - Transparent Huge Pages）。在扫描 PMD 表项时，如果检测到大页被访问，同样提升到最新 gen。

 * 场景 3：回收路径的反向映射邻近扫描

  ★ （同上的 kswapd / 直接回收 / memcg / NUMA 入口）
                    ↓
  shrink_node()                                   [mm/vmscan.c:7229]
    └─ lru_gen_shrink_node()                      [mm/vmscan.c:6130]
         └─ shrink_many()                         [mm/vmscan.c:6039]
              └─ shrink_one()                     [mm/vmscan.c:6018]
                   └─ try_to_shrink_lruvec()      [mm/vmscan.c:5976]
                        └─ evict_folios()         [mm/vmscan.c:5548]
                             └─ shrink_folio_list()  [mm/vmscan.c:1447]
                                  └─ lru_gen_look_around()  [mm/vmscan.c:5207]
                                       └─ folio_update_gen()  ← 调用点3 [vmscan.c:5290]
 * 场景3说明：
 *	在回收路径中，通过反向映射（rmap）检查目标 folio 周围的邻近页面。如果邻近页面也被访问过，一并提升到最新 gen，避免误驱逐热页。
 */


/* protect pages accessed multiple times through file descriptors */
/*
 * 保护页面，页面是指那些通过fd多次访问的页面，比如sys read
 * 为了在回收的路径中，保护页面不被驱逐
 *
 * 目标是将页面向前推进一代（向新的代推进），
 * 如果发现页面已经被移动到更新的代，则直接返回
 */
static int folio_inc_gen(struct lruvec *lruvec, struct folio *folio, bool reclaiming)
{
	int type = folio_is_file_lru(folio);
	struct lru_gen_folio *lrugen = &lruvec->lrugen;
	/*
	 * old_gen: 由min_seq获取当前MGLRU对应type中最老的gen
	 *
	 * 对于进入回收路径的folio，我们期望它所在的gen是最老的、或者说它所在的gen就应该是最老的
	 * 所以在后面的循环里会用这个最老的gen，来判断folio在后面有没有被移动移动到更新的gen
	 */
	int new_gen, old_gen = lru_gen_from_seq(lrugen->min_seq[type]);
	/* 进入下面的循环前，先保存folio->flags的快照，因为后续可能会被异步更新 */
	unsigned long new_flags, old_flags = READ_ONCE(folio->flags);

	VM_WARN_ON_ONCE_FOLIO(!(old_flags & LRU_GEN_MASK), folio);

	do {
		/* 从flags快照中获取folio所在的gen */
		new_gen = ((old_flags & LRU_GEN_MASK) >> LRU_GEN_PGOFF) - 1;
		/* folio_update_gen() has promoted this page? */
		/*
		 * 两个判断条件：
		 * 1.new_gen >= 0，说明folio仍在MGLRU中
		 *	    == -1，说明被隔离移除了(lru_gen_del_folio)
		 *
		 * 2.new_gen != old_gen，说明folio已经已经被移动到更新的gen了(异步提升)
		 *	     == old_gen，说明folio仍然在最老的gen
		 *
		 * 第一次走到这里，分几种情况：
		 * 1.new_gen == -1，这种情况基本不存在
		 *	-1表示页面不在MGLRU链表中了，可能的场景是在回收路径中被隔离（lru_gen_del_folio）, 而这个操作也是需要拿MGLRU锁的，所以对于同一个folio来说，不存在这种情况
		 *
		 * 2.new_gen >=0 并且 new_gen == old_gen
		 *	说明在这期间，页面没有被异步提升，所以继续往下执行，直接提升gen，下次循环到这里判断new_gen != old_gen后（因为被提升了，所以不相等），再退出
		 *
		 * 3.new_gen >=0 并且 new_gen != old_gen
		 *	说明在这期间，页面被被异步提升到最新的代（folio_update_gen），所以这里直接退出即可
		 */
		if (new_gen >= 0 && new_gen != old_gen)
			/*
			 * 直接返回new_gen，已经被提升到最新的代码了，调用者将其放到最新的代即可
			 * 不走下面的size统计更新，因为这个folio的gen没有变动
			 */
			return new_gen;

		/* gen+1 */
		new_gen = (old_gen + 1) % MAX_NR_GENS;

		/* 清除旧代号、旧 tier 信息，写入新代号 */
		new_flags = old_flags & ~(LRU_GEN_MASK | LRU_REFS_MASK | LRU_REFS_FLAGS);
		new_flags |= (new_gen + 1UL) << LRU_GEN_PGOFF;
		/* for folio_end_writeback() */
		/*
		 * 回收路径中对于正在writebakc的液界面，会设置reclaiming标志
		 * 这时候要设置PG_reclaim给wirteback最后阶段的folio_end_writebacak()判断条件是从
		 * 回写完成时，发现PG_reclaim被置位，则会将folio移动回正确的LRU位置（待研究）
		 */
		if (reclaiming)
			new_flags |= BIT(PG_reclaim);

	/*
	 * CSA，compare and swap，无锁原子操作
	 * static __always_inline bool
	 * atomic_long_try_cmpxchg_release(atomic_long_t *v, long *old, long new)
	 *	如果v 和 old的值相等，则将new的内容更新到v指向的内存中，返回true；
	 *	如果v 和 old的值不等，则v不变，将v的值更新到old指向的内存中，返回false；
	 */
	} while (!try_cmpxchg(&folio->flags, &old_flags, new_flags));
	/*
	 * folio gen变动，需要更新size统计
	 * 与 folio_update_gen 的关键区别：这里直接更新，不走批量 batch 路径
	 */
	lru_gen_update_size(lruvec, folio, old_gen, new_gen);

	return new_gen;
}
/*
  folio_update_gen（老化路径）和 folio_inc_gen（回收路径）可能并发作用于同一个 folio：

  CPU 0（老化路径）                    CPU 1（回收路径）
  walk_pte_range:                      sort_folio:
    发现 folio 在 gen=3（min_seq代）     扫到同一个 folio（gen=3，min_seq代）
    调用 folio_update_gen(folio, 2)      old_gen = lru_gen_from_seq(min_seq) = 3
    → CAS 将 flags 中代号改为 2+1=3     读 flags → 发现代号已经是 2+1=3（被 CPU0 改了）
                                         new_gen = 3-1 = 2，old_gen = 3
                                         new_gen(2) != old_gen(3) → 提前返回 2
                                         ← 不做任何操作，尊重老化路径的结果

  folio_inc_gen 的第 ⑥ 步竞争检测（new_gen != old_gen）就是专门处理这个 race 的。检测到 folio 已被老化路径提升后，直接返回新代号，不再重复操作。

  而 shrink_folio_list() 中也有一处对应处理（vmscan.c:1518）：

  // folio_update_gen() tried to promote this page?
  if (lru_gen_enabled() && !ignore_references &&
      folio_mapped(folio) && folio_test_referenced(folio))
      goto keep_locked;   // PG_referenced 由隔离后的 folio_update_gen 设置，保留不驱逐

  当 folio 已被隔离（LRU_GEN_MASK=0），folio_update_gen 无法更新代号，改为设置 PG_referenced；shrink_folio_list 看到这个 bit 后放弃驱逐，完成最后一道保护。


  ┌─────────────┬─────────────────────────────────────────────┬──────────────────────────────────────────┐
  │    维度     │              folio_update_gen               │              folio_inc_gen               │
  ├─────────────┼─────────────────────────────────────────────┼──────────────────────────────────────────┤
  │ 调用路径    │ 页表遍历（老化路径）                        │ 回收扫描 + min_seq 推进                  │
  ├─────────────┼─────────────────────────────────────────────┼──────────────────────────────────────────┤
  │ 目标代号    │ 调用方指定（总是 max_seq 对应 gen）         │ 固定 = old_gen + 1                       │
  ├─────────────┼─────────────────────────────────────────────┼──────────────────────────────────────────┤
  │ 跨代幅度    │ 可跨越多代（从任意旧代直达最新代）          │ 恰好 +1 代                               │
  ├─────────────┼─────────────────────────────────────────────┼──────────────────────────────────────────┤
  │ 大小统计    │ 不更新，延迟到 update_batch_size() 批量处理 │ 立即更新 lru_gen_update_size()           │
  ├─────────────┼─────────────────────────────────────────────┼──────────────────────────────────────────┤
  │ 需要 LRU 锁 │ 不需要（CAS + RCU 保护）                    │ 需要（调用方持有）                       │
  ├─────────────┼─────────────────────────────────────────────┼──────────────────────────────────────────┤
  │ 竞争检测    │ 无（它是"发起方"）                          │ 有（检测是否已被 folio_update_gen 提升） │
  ├─────────────┼─────────────────────────────────────────────┼──────────────────────────────────────────┤
  │ PG_reclaim  │ 不设置                                      │ reclaiming=true 时设置                   │
  └─────────────┴─────────────────────────────────────────────┴──────────────────────────────────────────┘

 * question:
 *	folio_inc_gen是把folio向前推进一代，也就是往新的代推进
 *	因为seq，新的代永远是在前面的，但是代码只是简单的+1，如果这时候gen是按照0、1、2、3排序的，也就是gen[0]是最老的，
 *	有没有可能对gen[3]的folio进行folio_gen_inc使得这个folio被移除gen[0], 也就是最老的一代？
 *
 * answer:
 *	不会，因为在folio_inc_gen中一旦检测到gen发生变化（不是最老的一代）,则直接退出
 */

static void update_batch_size(struct lru_gen_mm_walk *walk, struct folio *folio,
			      int old_gen, int new_gen)
{
	int type = folio_is_file_lru(folio);
	int zone = folio_zonenum(folio);
	int delta = folio_nr_pages(folio);

	VM_WARN_ON_ONCE(old_gen >= MAX_NR_GENS);
	VM_WARN_ON_ONCE(new_gen >= MAX_NR_GENS);

	walk->batched++;

	walk->nr_pages[old_gen][type][zone] -= delta;
	walk->nr_pages[new_gen][type][zone] += delta;
}

static void reset_batch_size(struct lru_gen_mm_walk *walk)
{
	int gen, type, zone;
	struct lruvec *lruvec = walk->lruvec;
	struct lru_gen_folio *lrugen = &lruvec->lrugen;

	walk->batched = 0;

	for_each_gen_type_zone(gen, type, zone) {
		enum lru_list lru = type * LRU_INACTIVE_FILE;
		int delta = walk->nr_pages[gen][type][zone];

		if (!delta)
			continue;

		walk->nr_pages[gen][type][zone] = 0;
		WRITE_ONCE(lrugen->nr_pages[gen][type][zone],
			   lrugen->nr_pages[gen][type][zone] + delta);

		if (lru_gen_is_active(lruvec, gen))
			lru += LRU_ACTIVE;
		__update_lru_size(lruvec, lru, zone, delta);
	}
}

static int should_skip_vma(unsigned long start, unsigned long end, struct mm_walk *args)
{
	struct address_space *mapping;
	struct vm_area_struct *vma = args->vma;
	struct lru_gen_mm_walk *walk = args->private;

	if (!vma_is_accessible(vma))
		return true;

	if (is_vm_hugetlb_page(vma))
		return true;

	if (!vma_has_recency(vma))
		return true;

	if (vma->vm_flags & (VM_LOCKED | VM_SPECIAL))
		return true;

	if (vma == get_gate_vma(vma->vm_mm))
		return true;

	if (vma_is_anonymous(vma))
		return !walk->can_swap;

	if (WARN_ON_ONCE(!vma->vm_file || !vma->vm_file->f_mapping))
		return true;

	mapping = vma->vm_file->f_mapping;
	if (mapping_unevictable(mapping))
		return true;

	if (shmem_mapping(mapping))
		return !walk->can_swap;

	/* to exclude special mappings like dax, etc. */
	return !mapping->a_ops->read_folio;
}

/*
 * Some userspace memory allocators map many single-page VMAs. Instead of
 * returning back to the PGD table for each of such VMAs, finish an entire PMD
 * table to reduce zigzags and improve cache performance.
 */
static bool get_next_vma(unsigned long mask, unsigned long size, struct mm_walk *args,
			 unsigned long *vm_start, unsigned long *vm_end)
{
	unsigned long start = round_up(*vm_end, size);
	unsigned long end = (start | ~mask) + 1;
	VMA_ITERATOR(vmi, args->mm, start);

	VM_WARN_ON_ONCE(mask & size);
	VM_WARN_ON_ONCE((start & mask) != (*vm_start & mask));

	for_each_vma(vmi, args->vma) {
		if (end && end <= args->vma->vm_start)
			return false;

		if (should_skip_vma(args->vma->vm_start, args->vma->vm_end, args))
			continue;

		*vm_start = max(start, args->vma->vm_start);
		*vm_end = min(end - 1, args->vma->vm_end - 1) + 1;

		return true;
	}

	return false;
}

static unsigned long get_pte_pfn(pte_t pte, struct vm_area_struct *vma, unsigned long addr,
				 struct pglist_data *pgdat)
{
	unsigned long pfn = pte_pfn(pte);

	VM_WARN_ON_ONCE(addr < vma->vm_start || addr >= vma->vm_end);

	if (!pte_present(pte) || is_zero_pfn(pfn))
		return -1;

	if (WARN_ON_ONCE(pte_devmap(pte) || pte_special(pte)))
		return -1;

	if (!pte_young(pte) && !mm_has_notifiers(vma->vm_mm))
		return -1;

	if (WARN_ON_ONCE(!pfn_valid(pfn)))
		return -1;

	if (pfn < pgdat->node_start_pfn || pfn >= pgdat_end_pfn(pgdat))
		return -1;

	return pfn;
}

static unsigned long get_pmd_pfn(pmd_t pmd, struct vm_area_struct *vma, unsigned long addr,
				 struct pglist_data *pgdat)
{
	unsigned long pfn = pmd_pfn(pmd);

	VM_WARN_ON_ONCE(addr < vma->vm_start || addr >= vma->vm_end);

	if (!pmd_present(pmd) || is_huge_zero_pmd(pmd))
		return -1;

	if (WARN_ON_ONCE(pmd_devmap(pmd)))
		return -1;

	if (!pmd_young(pmd) && !mm_has_notifiers(vma->vm_mm))
		return -1;

	if (WARN_ON_ONCE(!pfn_valid(pfn)))
		return -1;

	if (pfn < pgdat->node_start_pfn || pfn >= pgdat_end_pfn(pgdat))
		return -1;

	return pfn;
}

static struct folio *get_pfn_folio(unsigned long pfn, struct mem_cgroup *memcg,
				   struct pglist_data *pgdat, bool can_swap)
{
	struct folio *folio;

	folio = pfn_folio(pfn);
	if (folio_nid(folio) != pgdat->node_id)
		return NULL;

	if (folio_memcg_rcu(folio) != memcg)
		return NULL;

	/* file VMAs can contain anon pages from COW */
	if (!folio_is_file_lru(folio) && !can_swap)
		return NULL;

	return folio;
}

static bool suitable_to_scan(int total, int young)
{
	int n = clamp_t(int, cache_line_size() / sizeof(pte_t), 2, 8);

	/* suitable if the average number of young PTEs per cacheline is >=1 */
	return young * n >= total;
}

static bool walk_pte_range(pmd_t *pmd, unsigned long start, unsigned long end,
			   struct mm_walk *args)
{
	int i;
	pte_t *pte;
	spinlock_t *ptl;
	unsigned long addr;
	int total = 0;
	int young = 0;
	struct lru_gen_mm_walk *walk = args->private;
	struct mem_cgroup *memcg = lruvec_memcg(walk->lruvec);
	struct pglist_data *pgdat = lruvec_pgdat(walk->lruvec);
	DEFINE_MAX_SEQ(walk->lruvec);
	int old_gen, new_gen = lru_gen_from_seq(max_seq);

	pte = pte_offset_map_nolock(args->mm, pmd, start & PMD_MASK, &ptl);
	if (!pte)
		return false;
	if (!spin_trylock(ptl)) {
		pte_unmap(pte);
		return false;
	}

	arch_enter_lazy_mmu_mode();
restart:
	for (i = pte_index(start), addr = start; addr != end; i++, addr += PAGE_SIZE) {
		unsigned long pfn;
		struct folio *folio;
		pte_t ptent = ptep_get(pte + i);

		total++;
		walk->mm_stats[MM_LEAF_TOTAL]++;

		pfn = get_pte_pfn(ptent, args->vma, addr, pgdat);
		if (pfn == -1)
			continue;

		folio = get_pfn_folio(pfn, memcg, pgdat, walk->can_swap);
		if (!folio)
			continue;

		/*  读取页表的access bit，如果被访问过则继续往下，没被访问则返回检查下一个folio*/
		if (!ptep_clear_young_notify(args->vma, addr, pte + i))
			continue;

		young++;
		walk->mm_stats[MM_LEAF_YOUNG]++;

		if (pte_dirty(ptent) && !folio_test_dirty(folio) &&
		    !(folio_test_anon(folio) && folio_test_swapbacked(folio) &&
		      !folio_test_swapcache(folio)))
			folio_mark_dirty(folio);

		/* 页面被访问过，将其提升到最新的gen */
		old_gen = folio_update_gen(folio, new_gen);
		/* 如果folio原来就在new_gen, 则不需要更新size，反之则更新(延迟批量更新) */
		if (old_gen >= 0 && old_gen != new_gen)
			update_batch_size(walk, folio, old_gen, new_gen);
	}

	if (i < PTRS_PER_PTE && get_next_vma(PMD_MASK, PAGE_SIZE, args, &start, &end))
		goto restart;

	arch_leave_lazy_mmu_mode();
	pte_unmap_unlock(pte, ptl);

	return suitable_to_scan(total, young);
}

static void walk_pmd_range_locked(pud_t *pud, unsigned long addr, struct vm_area_struct *vma,
				  struct mm_walk *args, unsigned long *bitmap, unsigned long *first)
{
	int i;
	pmd_t *pmd;
	spinlock_t *ptl;
	struct lru_gen_mm_walk *walk = args->private;
	struct mem_cgroup *memcg = lruvec_memcg(walk->lruvec);
	struct pglist_data *pgdat = lruvec_pgdat(walk->lruvec);
	DEFINE_MAX_SEQ(walk->lruvec);
	int old_gen, new_gen = lru_gen_from_seq(max_seq);

	VM_WARN_ON_ONCE(pud_leaf(*pud));

	/* try to batch at most 1+MIN_LRU_BATCH+1 entries */
	if (*first == -1) {
		*first = addr;
		bitmap_zero(bitmap, MIN_LRU_BATCH);
		return;
	}

	i = addr == -1 ? 0 : pmd_index(addr) - pmd_index(*first);
	if (i && i <= MIN_LRU_BATCH) {
		__set_bit(i - 1, bitmap);
		return;
	}

	pmd = pmd_offset(pud, *first);

	ptl = pmd_lockptr(args->mm, pmd);
	if (!spin_trylock(ptl))
		goto done;

	arch_enter_lazy_mmu_mode();

	do {
		unsigned long pfn;
		struct folio *folio;

		/* don't round down the first address */
		addr = i ? (*first & PMD_MASK) + i * PMD_SIZE : *first;

		if (!pmd_present(pmd[i]))
			goto next;

		if (!pmd_trans_huge(pmd[i])) {
			if (!walk->force_scan && should_clear_pmd_young() &&
			    !mm_has_notifiers(args->mm))
				pmdp_test_and_clear_young(vma, addr, pmd + i);
			goto next;
		}

		pfn = get_pmd_pfn(pmd[i], vma, addr, pgdat);
		if (pfn == -1)
			goto next;

		folio = get_pfn_folio(pfn, memcg, pgdat, walk->can_swap);
		if (!folio)
			goto next;

		if (!pmdp_clear_young_notify(vma, addr, pmd + i))
			goto next;

		walk->mm_stats[MM_LEAF_YOUNG]++;

		if (pmd_dirty(pmd[i]) && !folio_test_dirty(folio) &&
		    !(folio_test_anon(folio) && folio_test_swapbacked(folio) &&
		      !folio_test_swapcache(folio)))
			folio_mark_dirty(folio);

		old_gen = folio_update_gen(folio, new_gen);
		if (old_gen >= 0 && old_gen != new_gen)
			update_batch_size(walk, folio, old_gen, new_gen);
next:
		i = i > MIN_LRU_BATCH ? 0 : find_next_bit(bitmap, MIN_LRU_BATCH, i) + 1;
	} while (i <= MIN_LRU_BATCH);

	arch_leave_lazy_mmu_mode();
	spin_unlock(ptl);
done:
	*first = -1;
}

static void walk_pmd_range(pud_t *pud, unsigned long start, unsigned long end,
			   struct mm_walk *args)
{
	int i;
	pmd_t *pmd;
	unsigned long next;
	unsigned long addr;
	struct vm_area_struct *vma;
	DECLARE_BITMAP(bitmap, MIN_LRU_BATCH);
	unsigned long first = -1;
	struct lru_gen_mm_walk *walk = args->private;
	struct lru_gen_mm_state *mm_state = get_mm_state(walk->lruvec);

	VM_WARN_ON_ONCE(pud_leaf(*pud));

	/*
	 * Finish an entire PMD in two passes: the first only reaches to PTE
	 * tables to avoid taking the PMD lock; the second, if necessary, takes
	 * the PMD lock to clear the accessed bit in PMD entries.
	 */
	pmd = pmd_offset(pud, start & PUD_MASK);
restart:
	/* walk_pte_range() may call get_next_vma() */
	vma = args->vma;
	for (i = pmd_index(start), addr = start; addr != end; i++, addr = next) {
		pmd_t val = pmdp_get_lockless(pmd + i);

		next = pmd_addr_end(addr, end);

		if (!pmd_present(val) || is_huge_zero_pmd(val)) {
			walk->mm_stats[MM_LEAF_TOTAL]++;
			continue;
		}

		if (pmd_trans_huge(val)) {
			struct pglist_data *pgdat = lruvec_pgdat(walk->lruvec);
			unsigned long pfn = get_pmd_pfn(val, vma, addr, pgdat);

			walk->mm_stats[MM_LEAF_TOTAL]++;

			if (pfn != -1)
				walk_pmd_range_locked(pud, addr, vma, args, bitmap, &first);
			continue;
		}

		if (!walk->force_scan && should_clear_pmd_young() &&
		    !mm_has_notifiers(args->mm)) {
			if (!pmd_young(val))
				continue;

			walk_pmd_range_locked(pud, addr, vma, args, bitmap, &first);
		}

		if (!walk->force_scan && !test_bloom_filter(mm_state, walk->seq, pmd + i))
			continue;

		walk->mm_stats[MM_NONLEAF_FOUND]++;

		if (!walk_pte_range(&val, addr, next, args))
			continue;

		walk->mm_stats[MM_NONLEAF_ADDED]++;

		/* carry over to the next generation */
		update_bloom_filter(mm_state, walk->seq + 1, pmd + i);
	}

	walk_pmd_range_locked(pud, -1, vma, args, bitmap, &first);

	if (i < PTRS_PER_PMD && get_next_vma(PUD_MASK, PMD_SIZE, args, &start, &end))
		goto restart;
}

static int walk_pud_range(p4d_t *p4d, unsigned long start, unsigned long end,
			  struct mm_walk *args)
{
	int i;
	pud_t *pud;
	unsigned long addr;
	unsigned long next;
	struct lru_gen_mm_walk *walk = args->private;

	VM_WARN_ON_ONCE(p4d_leaf(*p4d));

	pud = pud_offset(p4d, start & P4D_MASK);
restart:
	for (i = pud_index(start), addr = start; addr != end; i++, addr = next) {
		pud_t val = READ_ONCE(pud[i]);

		next = pud_addr_end(addr, end);

		if (!pud_present(val) || WARN_ON_ONCE(pud_leaf(val)))
			continue;

		walk_pmd_range(&val, addr, next, args);

		if (need_resched() || walk->batched >= MAX_LRU_BATCH) {
			end = (addr | ~PUD_MASK) + 1;
			goto done;
		}
	}

	if (i < PTRS_PER_PUD && get_next_vma(P4D_MASK, PUD_SIZE, args, &start, &end))
		goto restart;

	end = round_up(end, P4D_SIZE);
done:
	if (!end || !args->vma)
		return 1;

	walk->next_addr = max(end, args->vma->vm_start);

	return -EAGAIN;
}

static void walk_mm(struct mm_struct *mm, struct lru_gen_mm_walk *walk)
{
	static const struct mm_walk_ops mm_walk_ops = {
		.test_walk = should_skip_vma,
		.p4d_entry = walk_pud_range,
		.walk_lock = PGWALK_RDLOCK,
	};

	int err;
	struct lruvec *lruvec = walk->lruvec;
	struct mem_cgroup *memcg = lruvec_memcg(lruvec);

	walk->next_addr = FIRST_USER_ADDRESS;

	do {
		DEFINE_MAX_SEQ(lruvec);

		err = -EBUSY;

		/* another thread might have called inc_max_seq() */
		if (walk->seq != max_seq)
			break;

		/* folio_update_gen() requires stable folio_memcg() */
		if (!mem_cgroup_trylock_pages(memcg))
			break;

		/* the caller might be holding the lock for write */
		if (mmap_read_trylock(mm)) {
			err = walk_page_range(mm, walk->next_addr, ULONG_MAX, &mm_walk_ops, walk);

			mmap_read_unlock(mm);
		}

		mem_cgroup_unlock_pages();

		if (walk->batched) {
			spin_lock_irq(&lruvec->lru_lock);
			reset_batch_size(walk);
			spin_unlock_irq(&lruvec->lru_lock);
		}

		cond_resched();
	} while (err == -EAGAIN);
}

static struct lru_gen_mm_walk *set_mm_walk(struct pglist_data *pgdat, bool force_alloc)
{
	struct lru_gen_mm_walk *walk = current->reclaim_state->mm_walk;

	if (pgdat && current_is_kswapd()) {
		VM_WARN_ON_ONCE(walk);

		walk = &pgdat->mm_walk;
	} else if (!walk && force_alloc) {
		VM_WARN_ON_ONCE(current_is_kswapd());

		walk = kzalloc(sizeof(*walk), __GFP_HIGH | __GFP_NOMEMALLOC | __GFP_NOWARN);
	}

	current->reclaim_state->mm_walk = walk;

	return walk;
}

static void clear_mm_walk(void)
{
	struct lru_gen_mm_walk *walk = current->reclaim_state->mm_walk;

	VM_WARN_ON_ONCE(walk && memchr_inv(walk->nr_pages, 0, sizeof(walk->nr_pages)));
	VM_WARN_ON_ONCE(walk && memchr_inv(walk->mm_stats, 0, sizeof(walk->mm_stats)));

	current->reclaim_state->mm_walk = NULL;

	if (!current_is_kswapd())
		kfree(walk);
}

/*
 * 强制推进 min_seq[type]：将当前最老代（gen=min_seq%4）的所有 folio
 * 通过 folio_inc_gen 迁移到次老代（gen+1），然后 min_seq++。
 *
 * 使用场景：
 * 1. inc_max_seq() 内部：当代数已满（nr_gens==MAX_NR_GENS=4）时，
 *    必须先驱逐最老代的代槽，才能腾出空间给新代。
 * 2. 间接被 try_to_inc_min_seq() 调用（已验证无效情况直接 goto done）。
 *
 * 批量处理：每次最多迁移 MAX_LRU_BATCH 个 folio，避免长时间持锁，
 * 若未处理完则返回 false，调用者需循环重试。
 *
 * 调用者持有 lruvec->lru_lock。
 */
static bool inc_min_seq(struct lruvec *lruvec, int type, bool can_swap)
{
	int zone;
	int remaining = MAX_LRU_BATCH;
	struct lru_gen_folio *lrugen = &lruvec->lrugen;
	int new_gen, old_gen = lru_gen_from_seq(lrugen->min_seq[type]);

	if (type == LRU_GEN_ANON && !can_swap)
		goto done;

	/* prevent cold/hot inversion if force_scan is true */
	for (zone = 0; zone < MAX_NR_ZONES; zone++) {
		struct list_head *head = &lrugen->folios[old_gen][type][zone];

		while (!list_empty(head)) {
			struct folio *folio = lru_to_folio(head);

			VM_WARN_ON_ONCE_FOLIO(folio_test_unevictable(folio), folio);
			VM_WARN_ON_ONCE_FOLIO(folio_test_active(folio), folio);
			VM_WARN_ON_ONCE_FOLIO(folio_is_file_lru(folio) != type, folio);
			VM_WARN_ON_ONCE_FOLIO(folio_zonenum(folio) != zone, folio);

			/*
			 * 将folio的gen加1，并移动到新的gen链表中
			 */
			new_gen = folio_inc_gen(lruvec, folio, false);
			list_move_tail(&folio->lru, &lrugen->folios[new_gen][type][zone]);

			if (!--remaining)
				return false;
		}
	}
done:
	reset_ctrl_pos(lruvec, type, true);
	/* 推进min_seq */
	WRITE_ONCE(lrugen->min_seq[type], lrugen->min_seq[type] + 1);

	return true;
}

/*
 * 尝试"免费"推进 min_seq：不强制迁移 folio（不调用 folio_inc_gen），
 * 只扫描最老代 gen 槽是否已经完全为空，若为空则直接推进 min_seq。
 *
 * 语义：若最老代恰好没有任何 folio（自然清空，例如已被 eviction 驱逐完），
 * 则 min_seq 可以"跳过"这一代，不需要为每个 folio 单独做迁移操作。
 *
 * 调用点：evict_folios()（在持有 lru_lock 时调用），紧接在 isolate_folios 之后，
 * 利用刚刚批量隔离造成的"老代变空"机会推进 min_seq。
 *
 * 返回 true  = 至少推进了一个 type 的 min_seq；
 * 返回 false = 两个 type 均无法推进。
 */
static bool try_to_inc_min_seq(struct lruvec *lruvec, bool can_swap)
{
	int gen, type, zone;
	bool success = false;
	struct lru_gen_folio *lrugen = &lruvec->lrugen;
	DEFINE_MIN_SEQ(lruvec);

	VM_WARN_ON_ONCE(!seq_is_valid(lruvec));

       /*
        * 对每个 type 独立扫描：尝试将 min_seq[type] 向前推进，跳过空代。
        *
        * 起始 type：
        * - can_swap=false → !can_swap=1 → type 从 LRU_GEN_FILE(1) 开始，跳过 ANON
        * - can_swap=true  → !can_swap=0 → type 从 LRU_GEN_ANON(0) 开始，两个都扫
        */
	/* find the oldest populated generation */
	for (type = !can_swap; type < ANON_AND_FILE; type++) {
		while (min_seq[type] + MIN_NR_GENS <= lrugen->max_seq) {
			/* 当前 min_seq 对应的 gen 槽索引 */
			gen = lru_gen_from_seq(min_seq[type]);

			/* 检查该 gen 槽在所有 zone 中是否都没有 folio */
			for (zone = 0; zone < MAX_NR_ZONES; zone++) {
				if (!list_empty(&lrugen->folios[gen][type][zone]))
					goto next;
			}

			/* 所有 zone 均为空，此代可以丢弃，本地推进 min_seq */
			min_seq[type]++;
		}
next:
		;
	}

	/* see the comment on lru_gen_folio */
	if (can_swap) {
		min_seq[LRU_GEN_ANON] = min(min_seq[LRU_GEN_ANON], min_seq[LRU_GEN_FILE]);
		min_seq[LRU_GEN_FILE] = max(min_seq[LRU_GEN_ANON], lrugen->min_seq[LRU_GEN_FILE]);
	}

	for (type = !can_swap; type < ANON_AND_FILE; type++) {
		if (min_seq[type] == lrugen->min_seq[type])
			continue;

		reset_ctrl_pos(lruvec, type, true);
		WRITE_ONCE(lrugen->min_seq[type], min_seq[type]);
		success = true;
	}

	return success;
}

/*
 * 真正执行 MGLRU 老化的函数：将 max_seq 原子地加 1，创建一个新的最年轻代。
 *
 * 老化的本质：max_seq++ 使得原来的最新代变成次新代，新的最新代（max_seq 对应的
 * gen 槽）初始为空，等待新访问的页面被 folio_update_gen() 提升进来。
 *
 * inc_max_seq()目标增加一个最新的gen，而且是空的，等待新访问的页面被加进来;
 * 使得以前的最新代变老的，变为次新代了。
 *
 * 参数：
 *   lruvec     - 目标 lruvec（每个 memcg 每个 node 一个）
 *   seq        - 调用者读取的 max_seq 快照，用于并发检测
 *   can_swap   - 是否允许回收匿名页（影响 min_seq 推进策略）
 *   force_scan - 是否强制老化（即使代数已满 MAX_NR_GENS 也强制推进 min_seq）
 *
 * 返回值：
 *   true  - 本次老化成功，max_seq 已被本线程推进
 *   false - 老化失败，max_seq 已被其他线程推进（并发场景下的正常结果）
 */
static bool inc_max_seq(struct lruvec *lruvec, unsigned long seq,
			bool can_swap, bool force_scan)
{
	bool success;  /* 标记本次老化是否由本线程完成 */
	int prev, next;  /* prev: max_seq-1 对应的 gen 索引；next: max_seq+1 对应的 gen 索引 */
	int type, zone;  /* 循环变量：页面类型（ANON/FILE）和内存区域（zone） */
	struct lru_gen_folio *lrugen = &lruvec->lrugen;  /* 取 lruvec 的 MGLRU 核心结构 */
restart:
	/* 无锁乐观检查：若 max_seq 已超过快照值 seq，说明其他线程已完成老化，无需重复 */
	if (seq < READ_ONCE(lrugen->max_seq))
		return false;

	/* 加锁，准备正式执行老化（需要修改 max_seq 等共享数据） */
	spin_lock_irq(&lruvec->lru_lock);

	/* 调试断言：验证 seq 的合法性（FILE gens 在 [MIN_NR_GENS, MAX_NR_GENS] 内，ANON >= FILE） */
	VM_WARN_ON_ONCE(!seq_is_valid(lruvec));

	/* 加锁后二次确认：防止在乐观检查和加锁之间的窗口期内被其他线程抢先推进 */
	success = seq == lrugen->max_seq;
	/* max_seq 已被其他线程推进，本次老化不需要执行，跳到 unlock */
	if (!success)
		goto unlock;

	/*
	 * 检查是否存在代数已满（nr_gens == MAX_NR_GENS == 4）的情况。
	 * max_seq++ 要求 nr_gens < MAX_NR_GENS，否则新代会溢出 4 代上限。
	 * 若某个 type 已有 4 代，需先调用 inc_min_seq() 强制推进 min_seq，
	 * 将最老代的页面迁移到次老代，腾出一个代的槽位。
	 * 注意：倒序遍历（FILE 先于 ANON），确保文件页优先处理。
	 */
	for (type = ANON_AND_FILE - 1; type >= 0; type--) {
		/* 当前 type 的代数未满，无需处理，继续检查下一个 type */
		if (get_nr_gens(lruvec, type) != MAX_NR_GENS)
			continue;

		/*
		 * 健全性检查：正常回收路径下（非 force_scan），代数满说明回收严重滞后。
		 * FILE type 或允许 swap 时不应出现代满（回收应该及时推进 min_seq）。
		 */
		VM_WARN_ON_ONCE(!force_scan && (type == LRU_GEN_FILE || can_swap));

		/* 尝试推进 min_seq[type]，将最老代的页面批量迁移到次老代（folio_inc_gen） */
		/* inc_min_seq 每次最多处理 MAX_LRU_BATCH 个 folio，避免长时间持锁 */
		if (inc_min_seq(lruvec, type, can_swap))
			/* min_seq 推进成功，该 type 代数已降为 3，继续检查下一个 type */
			continue;

		/*
		 * inc_min_seq 返回 false：最老代页面过多，本批次未处理完。
		 * 释放锁，让出 CPU 后重新从 restart 开始，分批处理，避免软锁死。
		 */
		spin_unlock_irq(&lruvec->lru_lock);
		cond_resched();  /* 主动让出 CPU，避免长时间占用导致 soft lockup */
		goto restart;    /* 重新从乐观检查开始，确保 seq 仍然有效 */
	}

	/*
	 * Update the active/inactive LRU sizes for compatibility. Both sides of
	 * the current max_seq need to be covered, since max_seq+1 can overlap
	 * with min_seq[LRU_GEN_ANON] if swapping is constrained. And if they do
	 * overlap, cold/hot inversion happens.
	 */
	/*
	 * 计算 prev 和 next 的 gen 环形索引（gen = seq % MAX_NR_GENS）：
	 * prev = (max_seq - 1) % 4：当前次新代的 gen 槽索引
	 * next = (max_seq + 1) % 4：max_seq++ 后新最新代将占用的 gen 槽索引
	 * 注意：next 槽在 max_seq++ 后将成为新的最年轻代（初始为空），
	 * 而该槽此前可能存有上一轮回收后残留的旧数据（若 swap 受限导致冷热倒置）。
	 */
	prev = lru_gen_from_seq(lrugen->max_seq - 1);
	next = lru_gen_from_seq(lrugen->max_seq + 1);

	/*
	 * 同步更新传统 active/inactive LRU 计数，保持与内核其他子系统的兼容性。
	 * （/proc/meminfo、vmstat 等仍依赖传统的 LRU_ACTIVE_ANON 等计数）
	 * 逻辑：prev 代（即将成为次新代，视为 inactive）与 next 代（将被清空，
	 * 当前可能有残留旧数据）的页面数之差，需要同步反映到 inactive/active 计数中。
	 */
	for (type = 0; type < ANON_AND_FILE; type++) {
		for (zone = 0; zone < MAX_NR_ZONES; zone++) {
			/* lru 为传统 LRU 中该 type 对应的 inactive 链表枚举值 */
			enum lru_list lru = type * LRU_INACTIVE_FILE;
			/* delta = prev 代页数 - next 代页数（next 代正常情况下为 0） */
			long delta = lrugen->nr_pages[prev][type][zone] -
				     lrugen->nr_pages[next][type][zone];

			/* 差值为 0 无需更新，跳过 */
			if (!delta)
				continue;

			/* inactive 计数加 delta（prev 代视为 inactive） */
			__update_lru_size(lruvec, lru, zone, delta);
			/* active 计数减 delta（与 inactive 保持总量一致） */
			__update_lru_size(lruvec, lru + LRU_ACTIVE, zone, -delta);
		}
	}

	/* 重置 PID 控制器的历史统计数据，为新代建立干净的 refault/evict 统计起点 */
	for (type = 0; type < ANON_AND_FILE; type++)
		reset_ctrl_pos(lruvec, type, false);

	/* 记录新代（next 槽）的创建时间戳，供老化间隔统计使用 */
	WRITE_ONCE(lrugen->timestamps[next], jiffies);
	/* make sure preceding modifications appear */
	/*
	 * 这里真正将max_seq加1
	 *
	 * 使用 smp_store_release 发布屏障：确保前面所有对 lrugen 的修改
	 * 对其他 CPU 可见后，才发布新的 max_seq 值，防止乱序导致其他 CPU
	 * 看到新 max_seq 但看不到配套的统计更新。
	 */
	smp_store_release(&lrugen->max_seq, lrugen->max_seq + 1);
unlock:
	/* 释放 LRU 锁 */
	spin_unlock_irq(&lruvec->lru_lock);

	/* 返回本次老化是否由本线程完成（true=成功推进，false=被抢先） */
	return success;
}

/*
 * MGLRU 老化的入口函数：在调用 inc_max_seq() 推进 max_seq 之前，
 * 先遍历 mm_list（进程地址空间列表），扫描页表中的 accessed 位，
 * 将被访问的页面从旧代提升到新代，以便老化决策基于最新的访问信息。
 *
 * 参数：
 *   lruvec     - 目标 lruvec（对应一个 memcg × pgdat 的组合）
 *   seq        - 调用者观察到的 max_seq 值（用于并发保护）
 *   can_swap   - 是否允许扫描匿名页映射（swap 是否可用）
 *   force_scan - 是否强制扫描全部 mm（忽略 mm 的年龄判断）
 *
 * 返回值：true = 本线程完成了一次完整的老化（max_seq 已推进）；
 *         false = 被其他线程抢先完成，本线程未推进 max_seq。
 */
static bool try_to_inc_max_seq(struct lruvec *lruvec, unsigned long seq,
			       bool can_swap, bool force_scan)
{
	bool success;  /* 标记本线程是否完成了完整的 mm_list 遍历并成功推进 max_seq */
	struct lru_gen_mm_walk *walk;  /* 页表扫描的 walk 上下文（per-cpu 缓存分配） */
	struct mm_struct *mm = NULL;  /* 当前正在扫描的进程 mm_struct（NULL 表示尚未取到） */
	struct lru_gen_folio *lrugen = &lruvec->lrugen;  /* 本 lruvec 的 MGLRU 代数据 */
	/* mm_state：管理 mm_list 遍历进度的状态结构；仅当开启了 MGLRU mm tracking 时非 NULL */
	struct lru_gen_mm_state *mm_state = get_mm_state(lruvec);

	/* 断言：调用者传入的 seq 不能超过当前 max_seq（seq 只能等于或落后于 max_seq） */
	VM_WARN_ON_ONCE(seq > READ_ONCE(lrugen->max_seq));

	/*
	 * mm_state 为 NULL 表示此 lruvec 没有关联的 mm_list（例如内核线程专属 memcg、
	 * 或 CONFIG_LRU_GEN_WALKS_MMU 未开启），无需页表扫描，直接推进 max_seq。
	 */
	if (!mm_state)
		return inc_max_seq(lruvec, seq, can_swap, force_scan);

	/*
	 * mm_state->seq 记录上一次完整 mm_list 遍历时对应的 max_seq 值。
	 * 若 seq <= mm_state->seq，说明当前 seq 轮次的 mm_list 已被其他线程
	 * 遍历完毕（iterate_mm_list 已将 mm_state->seq 推进到 seq），
	 * 本线程无需重复扫描，返回 false 表示"已被抢先完成"。
	 * （详见 iterate_mm_list() 中的注释）
	 */
	/* see the comment in iterate_mm_list() */
	if (seq <= READ_ONCE(mm_state->seq))
		return false;

	/*
	 * If the hardware doesn't automatically set the accessed bit, fallback
	 * to lru_gen_look_around(), which only clears the accessed bit in a
	 * handful of PTEs. Spreading the work out over a period of time usually
	 * is less efficient, but it avoids bursty page faults.
	 */
	/*
	 * should_walk_mmu() 返回 false：硬件不自动维护 PTE accessed 位（如某些 ARM 平台），
	 * 主动扫描页表意义不大（accessed 位不可靠）。
	 * 退化为 iterate_mm_list_nowalk()：仅推进 mm_state->seq，不真正扫描页表，
	 * 由 lru_gen_look_around()（缺页中断路径）负责少量 PTE 的 accessed 位清理，
	 * 虽效率略低，但可避免集中扫描带来的缺页中断突发。
	 */
	if (!should_walk_mmu()) {
		success = iterate_mm_list_nowalk(lruvec, seq);  /* 仅更新 mm_state->seq，不扫页表 */
		goto done;
	}

	/*
	 * 从 per-cpu 缓存分配 lru_gen_mm_walk 上下文。
	 * 第二个参数 true 表示允许阻塞分配（direct reclaim 路径），
	 * false 则只使用 per-cpu 缓存（kswapd 路径可能传 false）。
	 */
	walk = set_mm_walk(NULL, true);
	if (!walk) {
		/* 分配失败（内存紧张），退化为 nowalk 路径 */
		success = iterate_mm_list_nowalk(lruvec, seq);
		goto done;
	}

	/* 初始化 walk 上下文，绑定目标 lruvec 及本次老化的参数 */
	walk->lruvec = lruvec;      /* 目标 lruvec，walk_mm 将把访问到的页面提升至此 lruvec */
	walk->seq = seq;            /* 本次老化对应的 max_seq，防止跨轮次错误提升 */
	walk->can_swap = can_swap;  /* 是否扫描匿名页映射（需 swap 支持） */
	walk->force_scan = force_scan;  /* 是否忽略 mm 的时间戳，强制全量扫描 */

	/*
	 * 逐个取出 memcg->mm_list 中的 mm_struct 并扫描其页表：
	 *
	 * - iterate_mm_list()：从 memcg的mm_list FIFO 队列取下一个 mm，
	 *   若本轮遍历已完成（所有 mm 均处理完毕）则返回 success=true，
	 *   同时将 mm_state->seq 推进到 seq，防止其他线程重复扫描；
	 *   mm_list FIFO链表存放的是所有隶属于该memcg的、仍在运行的用户进程的地址空间（内核线程没有mm，不在链表中）
	 *   相关操作接口：
	 *   ┌─────────────────────────────────────────────┐
	 *   │          操作          │    调用点    │                    触发时机                    │
	 *   ├─────────────────────────────────────────────┤
	 *   │ lru_gen_add_mm(mm)     │ mm_init()    │ 进程创建时，mm 初始化后加入 memcg 的 mm_list   │
	 *   ├─────────────────────────────────────────────┤
	 *   │ lru_gen_del_mm(mm)     │ __mmdrop()   │ 进程退出，mm 引用计数归零时移出                │
	 *   ├─────────────────────────────────────────────┤
	 *   │ lru_gen_migrate_mm(mm) │ memcg 迁移时 │ 进程被移入新 memcg，mm 在新旧 mm_list 之间迁移 │
	 *   └────────────────────────┴────────────────────┘
	 *
	 * - walk_mm()：对取到的 mm 进行页表扫描，将 accesse bit被置位 对应的
	 *   folio 通过 folio_update_gen() 提升到最新代（max_seq % MAX_NR_GENS）；
	 *
	 * - 循环直到 iterate_mm_list() 返回 mm=NULL（本批次无更多 mm 可处理）。
	 *   其中iterate_mm_list()-->get_next_mm()会跳过哪些近期没有运行的task的mm
	 */
	do {
		success = iterate_mm_list(walk, &mm);  /* 取下一个 mm；success=true 表示完整遍历完成 */
		if (mm)
			walk_mm(mm, walk);  /* 扫描该 mm 的页表，更新 accessed folio 的 gen */
	} while (mm);  /* mm=NULL 时退出循环（当前批次已处理完） */
done:
	/*
	 * 只有在完成完整的 mm_list 遍历（success=true）之后，才调用 inc_max_seq()
	 * 真正推进 max_seq。这保证老化决策基于最新的页表访问信息。
	 * 若本线程未完成完整遍历（success=false），则不推进 max_seq，
	 * 留待下一轮调用继续处理剩余 mm。
	 */
	if (success) {
		/* 完整遍历后推进 max_seq；正常情况下 inc_max_seq 必须成功 */
		success = inc_max_seq(lruvec, seq, can_swap, force_scan);
		/* inc_max_seq 失败说明逻辑错误（seq 已被他人推进），触发告警 */
		WARN_ON_ONCE(!success);
	}
	/*
	 * 在推进 max_seq（创建新代）之前，先扫描所有进程的页表 PTE，
	 * 把 accessed bit 被置位的页面提前从旧 gen 迁移到新代（folio_update_gen(folio, max_seq % MAX_NR_GENS)），
	 * 这样做可以避免最老gen中那些最近被访问过的页面被误回收

		walk_mm() 扫描 PTE 时发现某页 accessed=1：
			 ↓
		folio 原在 gen0（最老）
			 ↓ folio_update_gen(folio, gen=3)
		更新folio flags的LRU_GEN(gen=3)
			 ↓ evict_folios()-->isolate_folios()-->scan_folios()-->sort_folios()移动folio
		folio 被移到 gen3, 最新链表

		之后回收路扫描gen0：此时这些folio已不在gen0，不会被误回收

	* 这使 "总是回收最老 gen" 的策略变得精确。
	*/

	/* 返回本线程是否成功完成了一次完整老化（推进了 max_seq） */
	return success;
}

/******************************************************************************
 *                          working set protection
 ******************************************************************************/

/*
 * 重新计算sc->priority的值
 */
static void set_initial_priority(struct pglist_data *pgdat, struct scan_control *sc)
{
	int priority;
	unsigned long reclaimable;

	if (sc->priority != DEF_PRIORITY || sc->nr_to_reclaim < MIN_LRU_BATCH)
		return;
	/*
	 * Determine the initial priority based on
	 * (total >> priority) * reclaimed_to_scanned_ratio = nr_to_reclaim,
	 * where reclaimed_to_scanned_ratio = inactive / total.
	 */
	/* 获取不活跃文件页面数 */
	reclaimable = node_page_state(pgdat, NR_INACTIVE_FILE);
	/* 如果可以回收匿名页面，则加上不活跃匿名页面数 */
	if (can_reclaim_anon_pages(NULL, pgdat->node_id, sc))
		reclaimable += node_page_state(pgdat, NR_INACTIVE_ANON);

	/* round down reclaimable and round up sc->nr_to_reclaim */
	priority = fls_long(reclaimable) - 1 - fls_long(sc->nr_to_reclaim - 1);

	/*
	 * The estimation is based on LRU pages only, so cap it to prevent
	 * overshoots of shrinker objects by large margins.
	 */
	/* 将sc->priority的值限制在DEF_PRIORITY / 2 到 DEF_PRIORITY之间 */
	sc->priority = clamp(priority, DEF_PRIORITY / 2, DEF_PRIORITY);
}

static bool lruvec_is_sizable(struct lruvec *lruvec, struct scan_control *sc)
{
	int gen, type, zone;
	unsigned long total = 0;
	bool can_swap = get_swappiness(lruvec, sc);
	struct lru_gen_folio *lrugen = &lruvec->lrugen;
	struct mem_cgroup *memcg = lruvec_memcg(lruvec);
	DEFINE_MAX_SEQ(lruvec);
	DEFINE_MIN_SEQ(lruvec);

	/*
	 * 待研究 MGLRU具体实现
	 * tier, gen, seq
	 * mglru关联数据结构： 以memcg为中心，核心数据结构是lru_gen_folio
	 */
	for (type = !can_swap; type < ANON_AND_FILE; type++) {
		unsigned long seq;

		for (seq = min_seq[type]; seq <= max_seq; seq++) {
			gen = lru_gen_from_seq(seq);

			for (zone = 0; zone < MAX_NR_ZONES; zone++)
				total += max(READ_ONCE(lrugen->nr_pages[gen][type][zone]), 0L);
		}
	}

	/* whether the size is big enough to be helpful */
	return mem_cgroup_online(memcg) ? (total >> sc->priority) : total;
}

/*
 * 检查传入的lruvec是否可回收
 * 基于内存保护设置、规模大小和页面年龄进行综合判断
 */
static bool lruvec_is_reclaimable(struct lruvec *lruvec, struct scan_control *sc,
				  unsigned long min_ttl)
{
	int gen;
	unsigned long birth;  /* 最老页面的出生时间 */
	struct mem_cgroup *memcg = lruvec_memcg(lruvec); /* 根据lruvec获取memcg */
	DEFINE_MIN_SEQ(lruvec);	/* 定义最小代的序列号 */

        /* 如果memcg低于最低保护限制，不可回收 */
	if (mem_cgroup_below_min(NULL, memcg))
		return false;

        /*
	 * 检查LRU规模是否足够大, 如果不够大，则不可回收
	 * 重点：包含了seq和gen的操作
	 */
	if (!lruvec_is_sizable(lruvec, sc))
		return false;

	/* see the comment on lru_gen_folio */
        /* 参见struct lru_gen_folio的注释：获取文件页最老代际的出生时间点 */
	gen = lru_gen_from_seq(min_seq[LRU_GEN_FILE]);
	birth = READ_ONCE(lruvec->lrugen.timestamps[gen]);

	/*
	 * 检查最老页面的存活时间是否超过min_ttl，如果是则可回收
	 * 
	 * 注意：这里只是检查，并没有实际推荐MGLRU老化！
	 */
	return time_is_before_jiffies(birth + min_ttl);
}

/* to protect the working set of the last N jiffies */
static unsigned long lru_gen_min_ttl __read_mostly;

/*
 * MGLRU页面老化评估，包括匿名页面和文件页面
 */
static void lru_gen_age_node(struct pglist_data *pgdat, struct scan_control *sc)
{
	struct mem_cgroup *memcg;
	unsigned long min_ttl = READ_ONCE(lru_gen_min_ttl);
	bool reclaimable = !min_ttl;

	VM_WARN_ON_ONCE(!current_is_kswapd());

	/* 根据inactive file和inactive anon，重新计算sc->priority */
	set_initial_priority(pgdat, sc);

	/* 从root memcg开始，遍历所有memcg */
	memcg = mem_cgroup_iter(NULL, NULL, NULL);
	do {
		/* 根据memcg和node, 获取对应的lruvec */
		struct lruvec *lruvec = mem_cgroup_lruvec(memcg, pgdat);

		/*
		 * 计算当前memcg的内存保护设置
		 * 待研究: page_counter
		 */
		mem_cgroup_calculate_protection(NULL, memcg);

		if (!reclaimable)
			/* 评估当前memcg的内存是否可回收 */
			reclaimable = lruvec_is_reclaimable(lruvec, sc, min_ttl);
	} while ((memcg = mem_cgroup_iter(NULL, memcg, NULL)));

	/*
	 * The main goal is to OOM kill if every generation from all memcgs is
	 * younger than min_ttl. However, another possibility is all memcgs are
	 * either too small or below min.
	 */
	/*
         * 主要目标：如果所有memcg的所有代的存活时间都小于min_ttl，则触发OOM kill
	 *
         * 另一种可能性是所有memcg要么太小，要么低于最低保护限制
         */
	if (!reclaimable && mutex_trylock(&oom_lock)) {
		struct oom_control oc = {
			.gfp_mask = sc->gfp_mask,
		};

		out_of_memory(&oc);

		mutex_unlock(&oom_lock);
	}
}

/******************************************************************************
 *                          rmap/PT walk feedback
 ******************************************************************************/

/*
 * This function exploits spatial locality when shrink_folio_list() walks the
 * rmap. It scans the adjacent PTEs of a young PTE and promotes hot pages. If
 * the scan was done cacheline efficiently, it adds the PMD entry pointing to
 * the PTE table to the Bloom filter. This forms a feedback loop between the
 * eviction and the aging.
 */
/*
 * 在回收路径中，通过反向映射（rmap）检查目标 folio 周围的邻近页面。
 * 如果邻近页面也被访问过，一并提升到最新 gen，大大减少rmap vma遍历和页面的查询。
 *
  shrink_node()
    └─ lru_gen_shrink_node()
         └─ shrink_many()
              └─ shrink_one()
                   └─ try_to_shrink_lruvec()
                        └─ evict_folios()
                             └─ shrink_folio_list()
                                  └─ lru_gen_look_around()
                                       └─ folio_update_gen()
  shrink_folio_list()
    -->folio_check_references()
       -->folio_referenced()
	   -->folio_referenced_one()
	       -->lru_gen_look_around()
 */
bool lru_gen_look_around(struct page_vma_mapped_walk *pvmw)
{
	int i;
	unsigned long start;
	unsigned long end;
	struct lru_gen_mm_walk *walk;
	int young = 1;
	pte_t *pte = pvmw->pte;
	unsigned long addr = pvmw->address;
	struct vm_area_struct *vma = pvmw->vma;
	struct folio *folio = pfn_folio(pvmw->pfn);
	bool can_swap = !folio_is_file_lru(folio);
	struct mem_cgroup *memcg = folio_memcg(folio);
	struct pglist_data *pgdat = folio_pgdat(folio);
	struct lruvec *lruvec = mem_cgroup_lruvec(memcg, pgdat);
	struct lru_gen_mm_state *mm_state = get_mm_state(lruvec);
	DEFINE_MAX_SEQ(lruvec);
	int old_gen, new_gen = lru_gen_from_seq(max_seq);

	lockdep_assert_held(pvmw->ptl);
	VM_WARN_ON_ONCE_FOLIO(folio_test_lru(folio), folio);

	if (!ptep_clear_young_notify(vma, addr, pte))
		return false;

	if (spin_is_contended(pvmw->ptl))
		return true;

	/* exclude special VMAs containing anon pages from COW */
	if (vma->vm_flags & VM_SPECIAL)
		return true;

	/* avoid taking the LRU lock under the PTL when possible */
	walk = current->reclaim_state ? current->reclaim_state->mm_walk : NULL;

	start = max(addr & PMD_MASK, vma->vm_start);
	end = min(addr | ~PMD_MASK, vma->vm_end - 1) + 1;

	if (end - start == PAGE_SIZE)
		return true;

	if (end - start > MIN_LRU_BATCH * PAGE_SIZE) {
		if (addr - start < MIN_LRU_BATCH * PAGE_SIZE / 2)
			end = start + MIN_LRU_BATCH * PAGE_SIZE;
		else if (end - addr < MIN_LRU_BATCH * PAGE_SIZE / 2)
			start = end - MIN_LRU_BATCH * PAGE_SIZE;
		else {
			start = addr - MIN_LRU_BATCH * PAGE_SIZE / 2;
			end = addr + MIN_LRU_BATCH * PAGE_SIZE / 2;
		}
	}

	/* folio_update_gen() requires stable folio_memcg() */
	if (!mem_cgroup_trylock_pages(memcg))
		return true;

	arch_enter_lazy_mmu_mode();

	pte -= (addr - start) / PAGE_SIZE;

	for (i = 0, addr = start; addr != end; i++, addr += PAGE_SIZE) {
		unsigned long pfn;
		pte_t ptent = ptep_get(pte + i);

		pfn = get_pte_pfn(ptent, vma, addr, pgdat);
		if (pfn == -1)
			continue;

		folio = get_pfn_folio(pfn, memcg, pgdat, can_swap);
		if (!folio)
			continue;

		if (!ptep_clear_young_notify(vma, addr, pte + i))
			continue;

		young++;

		if (pte_dirty(ptent) && !folio_test_dirty(folio) &&
		    !(folio_test_anon(folio) && folio_test_swapbacked(folio) &&
		      !folio_test_swapcache(folio)))
			folio_mark_dirty(folio);

		if (walk) {
			old_gen = folio_update_gen(folio, new_gen);
			if (old_gen >= 0 && old_gen != new_gen)
				update_batch_size(walk, folio, old_gen, new_gen);

			continue;
		}

		old_gen = folio_lru_gen(folio);
		if (old_gen < 0)
			folio_set_referenced(folio);
		else if (old_gen != new_gen)
			folio_activate(folio);
	}

	arch_leave_lazy_mmu_mode();
	mem_cgroup_unlock_pages();

	/* feedback from rmap walkers to page table walkers */
	if (mm_state && suitable_to_scan(i, young))
		update_bloom_filter(mm_state, max_seq, pvmw->pmd);

	return true;
}

/******************************************************************************
 *                          memcg LRU
 ******************************************************************************/

/* see the comment on MEMCG_NR_GENS */
enum {
	MEMCG_LRU_NOP,
	MEMCG_LRU_HEAD,
	MEMCG_LRU_TAIL,
	MEMCG_LRU_OLD,
	MEMCG_LRU_YOUNG,
};

/*
 * MGLRU memcg划分为old\young两代的关键
 */
static void lru_gen_rotate_memcg(struct lruvec *lruvec, int op)
{
	int seg;
	int old, new;
	unsigned long flags;
	int bin = get_random_u32_below(MEMCG_NR_BINS);
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);

	spin_lock_irqsave(&pgdat->memcg_lru.lock, flags);

	VM_WARN_ON_ONCE(hlist_nulls_unhashed(&lruvec->lrugen.list));

	/*
	 * seg初始值为0
	 * 设置new初始值为old
	 */
	seg = 0;
	new = old = lruvec->lrugen.gen;

	/* see the comment on MEMCG_NR_GENS */
	/*
	 ┌────────────────────────────────────
	 │      操作       │        目标位置         │ gen 变化 │ seg 变化    │
	 ├────────────────────────────────────
	 │ MEMCG_LRU_HEAD  │ 当前代的随机 bin 头部   │ 不变     │ → HEAD     │
	 ├────────────────────────────────────
	 │ MEMCG_LRU_TAIL  │ 当前代的随机 bin 尾部   │ 不变     │ → TAIL     │
	 ├────────────────────────────────────
	 │ MEMCG_LRU_OLD   │ old 代的随机 bin 头部   │ → old    │ → default │
	 ├────────────────────────────────────
	 │ MEMCG_LRU_YOUNG │ young 代的随机 bin 尾部 │ → young  │ → default │
	 └────────────────────────────────────
	 */
	if (op == MEMCG_LRU_HEAD)
		seg = MEMCG_LRU_HEAD;
	else if (op == MEMCG_LRU_TAIL)
		seg = MEMCG_LRU_TAIL;
	else if (op == MEMCG_LRU_OLD)
		new = get_memcg_gen(pgdat->memcg_lru.seq);
	else if (op == MEMCG_LRU_YOUNG)
		new = get_memcg_gen(pgdat->memcg_lru.seq + 1);
	else
		VM_WARN_ON_ONCE(true);

	WRITE_ONCE(lruvec->lrugen.seg, seg);
	WRITE_ONCE(lruvec->lrugen.gen, new);

	/* 先将当前memcg从原来的链表中删除 */
	hlist_nulls_del_rcu(&lruvec->lrugen.list);

	/* 根据op要求，将memcg放入新的链表位置 */
	if (op == MEMCG_LRU_HEAD || op == MEMCG_LRU_OLD)
		hlist_nulls_add_head_rcu(&lruvec->lrugen.list, &pgdat->memcg_lru.fifo[new][bin]);
	else
		hlist_nulls_add_tail_rcu(&lruvec->lrugen.list, &pgdat->memcg_lru.fifo[new][bin]);

	pgdat->memcg_lru.nr_memcgs[old]--;
	pgdat->memcg_lru.nr_memcgs[new]++;

	/*
	 * 关键！！
	 * 只有等old代为空时，再向前推进memcg_lru.seq，这样就能保证永远最多只有两个代：old和young
	 * 推进后，以前的young代变为old，新上线的memcg或者rotote的memcg挂到新的young代中
	 */
	if (!pgdat->memcg_lru.nr_memcgs[old] && old == get_memcg_gen(pgdat->memcg_lru.seq))
		WRITE_ONCE(pgdat->memcg_lru.seq, pgdat->memcg_lru.seq + 1);

	spin_unlock_irqrestore(&pgdat->memcg_lru.lock, flags);
}

#ifdef CONFIG_MEMCG

/*
 * 用户态: mkdir /sys/fs/cgroup/foo
            │
            ▼
    cgroup_mkdir()                           [kernel/cgroup/cgroup.c]
      └── css_create()
            └── mem_cgroup_css_alloc()       ← 第一步：分配与初始化
                  └── mem_cgroup_alloc(parent)
                        ├─ kzalloc(struct_size(memcg, nodeinfo, nr_node_ids))
                        │     按节点数动态分配，nodeinfo[] 柔性数组
                        │
                        ├─ xa_alloc(&mem_cgroup_ids, &memcg->id.id, ...)
                        │     分配全局唯一 ID（1 ~ MEM_CGROUP_ID_MAX = 65535）
                        │
                        ├─ kzalloc(memcg_vmstats)          统计汇总结构
                        ├─ alloc_percpu(vmstats_percpu)    per-CPU 热统计
                        │     → 建立 statc->parent 链（指向父 memcg 的 percpu 统计）
                        │       用于 rstat 层次折叠
                        │
                        ├─ for_each_node: alloc_mem_cgroup_per_node_info()
                        │     每个 NUMA node 分配 mem_cgroup_per_node：
                        │       ├─ alloc_percpu(lruvec_stats_percpu)
                        │       └─ lruvec 本身嵌入其中（无需单独分配）
                        │
                        ├─ memcg_wb_domain_init()           writeback domain
                        ├─ INIT_WORK(&high_work, high_work_func)
                        ├─ vmpressure_init(&vmpressure)
                        └─ lru_gen_init_memcg(memcg)
                              └─ INIT_LIST_HEAD(&mm_list->fifo)
                                 spin_lock_init(&mm_list->lock)
                                 （MGLRU 的 mm_struct 扫描队列初始化）

                  page_counter_init(&memcg->memory, &parent->memory, true)
                      └─ counter->parent = &parent->memory
                         counter->max = PAGE_COUNTER_MAX   ← 初始无限制
                         （memory/swap/kmem 各建一棵向上指的计数树）

            └── mem_cgroup_css_online()      ← 第二步：发布使其可见
                  ├─ memcg_online_kmem()          分配 kmemcg_id，激活 slab 计量
                  ├─ alloc_shrinker_info()        分配 per-memcg shrinker bitmap
                  ├─ lru_gen_online_memcg()       ← MGLRU 关键步骤
                  │     for_each_node:
                  │       gen = get_memcg_gen(pgdat->memcg_lru.seq)
                  │       hlist_nulls_add_tail_rcu(&lruvec->lrugen.list,
                  │           &pgdat->memcg_lru.fifo[gen][bin])
                  │       pgdat->memcg_lru.nr_memcgs[gen]++
                  │     → 将新 memcg 的 lruvec 插入全局 memcg LRU 轮转队列
                  │       回收路径通过此队列轮询所有 memcg
                  │
                  ├─ refcount_set(&memcg->id.ref, 1)
                  ├─ css_get(css)                 id 持有一个 css 引用
                  └─ xa_store(&mem_cgroup_ids, id, memcg)
                         → 发布到 xarray，mem_cgroup_from_id() 从此可用


 * 用户态创建memcg时，最后将memcg上线
 *	css_create
	  --> mem_cgroup_alloc
	  --> mem_cgroup_css_online
	      --> lru_gen_onlien_memcg

 */
void lru_gen_online_memcg(struct mem_cgroup *memcg)
{
	int gen;
	int nid;
	/* 随机选择一个bin，0-7之间 */
	int bin = get_random_u32_below(MEMCG_NR_BINS);

	/*
	 * 遍历所有NUMA node, 将node对应的memcg挂入相应的链表
	 * 有多少个NUMA node就有多少个memcg
	 */
	for_each_node(nid) {
		struct pglist_data *pgdat = NODE_DATA(nid);
		struct lruvec *lruvec = get_lruvec(memcg, nid);

		spin_lock_irq(&pgdat->memcg_lru.lock);

		VM_WARN_ON_ONCE(!hlist_nulls_unhashed(&lruvec->lrugen.list));

		/* 获取memcg数组的old代 */
		gen = get_memcg_gen(pgdat->memcg_lru.seq);

		lruvec->lrugen.gen = gen;

		/* 将当前memcg挂入 old代的第bin条链表的尾部 */
		hlist_nulls_add_tail_rcu(&lruvec->lrugen.list, &pgdat->memcg_lru.fifo[gen][bin]);
		/* 统计old代的memcg计数 */
		pgdat->memcg_lru.nr_memcgs[gen]++;

		spin_unlock_irq(&pgdat->memcg_lru.lock);
	}
}

/*
 * 用户移除memcg时，对memcg进行下线
 *
  rmdir /sys/fs/cgroup/foo
    │
    ├─ css_offline()          ← 异步，引用归零前调用
    │     ├─ page_counter_set_min/low = 0    清除保护，避免阻止全局回收
    │     ├─ drain_all_stock()               回收所有 CPU stock，计数归精确值
    │     ├─ lru_gen_offline_memcg()         将 lruvec 标记为 OLD 代，加速回收
    │     ├─ wb_memcg_offline()              解除 writeback domain
    │     └─ mem_cgroup_id_put()             释放 id 引用
    │
    ├─ css_released()                        lruvec 回收迭代器失效
    │     └─ mem_cgroup_css_released
    │        └─ lru_gen_release_memcg
    └─ css_free()                            引用为 0 时调用
          ├─ lru_gen_exit_memcg()            释放 bloom filter 位图
          ├─ free_percpu(vmstats_percpu)
          ├─ kfree(vmstats)
          ├─ for_each_node: free_mem_cgroup_per_node_info()
          └─ kfree(memcg)

 */
void lru_gen_offline_memcg(struct mem_cgroup *memcg)
{
	int nid;

	for_each_node(nid) {
		struct lruvec *lruvec = get_lruvec(memcg, nid);

		/*
		 * 将memcg挂入到memcg old代链表的头部，后面会优先回收
		 */
		lru_gen_rotate_memcg(lruvec, MEMCG_LRU_OLD);
	}
}

void lru_gen_release_memcg(struct mem_cgroup *memcg)
{
	int gen;
	int nid;

	for_each_node(nid) {
		struct pglist_data *pgdat = NODE_DATA(nid);
		struct lruvec *lruvec = get_lruvec(memcg, nid);

		spin_lock_irq(&pgdat->memcg_lru.lock);

		if (hlist_nulls_unhashed(&lruvec->lrugen.list))
			goto unlock;

		gen = lruvec->lrugen.gen;

		/* 从链表中删除 */
		hlist_nulls_del_init_rcu(&lruvec->lrugen.list);
		pgdat->memcg_lru.nr_memcgs[gen]--;

		if (!pgdat->memcg_lru.nr_memcgs[gen] && gen == get_memcg_gen(pgdat->memcg_lru.seq))
			WRITE_ONCE(pgdat->memcg_lru.seq, pgdat->memcg_lru.seq + 1);
unlock:
		spin_unlock_irq(&pgdat->memcg_lru.lock);
	}
}

void lru_gen_soft_reclaim(struct mem_cgroup *memcg, int nid)
{
	struct lruvec *lruvec = get_lruvec(memcg, nid);

	/* see the comment on MEMCG_NR_GENS */
	if (READ_ONCE(lruvec->lrugen.seg) != MEMCG_LRU_HEAD)
		lru_gen_rotate_memcg(lruvec, MEMCG_LRU_HEAD);
}

#endif /* CONFIG_MEMCG */

/******************************************************************************
 *                          the eviction
 ******************************************************************************/

/*
 * 进到这里的页面，都是最老一代的页面
 */
static bool sort_folio(struct lruvec *lruvec, struct folio *folio, struct scan_control *sc,
		       int tier_idx)
{
	bool success;
	int gen = folio_lru_gen(folio);
	int type = folio_is_file_lru(folio);
	int zone = folio_zonenum(folio);
	int delta = folio_nr_pages(folio);
	int refs = folio_lru_refs(folio);
	int tier = lru_tier_from_refs(refs);
	struct lru_gen_folio *lrugen = &lruvec->lrugen;

	VM_WARN_ON_ONCE_FOLIO(gen >= MAX_NR_GENS, folio);

	/* unevictable */
	/* unevictable从链表删除，设置PG_unevictable */
	if (!folio_evictable(folio)) {
		success = lru_gen_del_folio(lruvec, folio, true);
		VM_WARN_ON_ONCE_FOLIO(!success, folio);
		folio_set_unevictable(folio);
		lruvec_add_folio(lruvec, folio);
		__count_vm_events(UNEVICTABLE_PGCULLED, delta);
		return true;
	}

	/* promoted */
	/*
	 * 如果页面已经不是最老的gen, 说明页面已经被异步提升到最新的gen，则直接将其放入对应的MGLRU链表
	 * get_nr_to_scan()->try_to_inc_max_seq()->walk_mm()-> ...->folio_update_gen()
	 */
	if (gen != lru_gen_from_seq(lrugen->min_seq[type])) {
		list_move(&folio->lru, &lrugen->folios[gen][type][zone]);
		return true;
	}

	/* protected */
	/*
	 * tier_idx: 可被驱逐的最大tier, 来自:
	 *	isolate_folios -> tier_idx = get_tier_idx(lruvec, type)
	 *		       -> scan_folios(, tier_idx) -> sort_folios(tier_idx)
	 * 超过tier_idx的页面如果被回收，refault的概率会很高
	 *
	 * 如果当前folio的tier大于tier_dix，说明这个folio不可以被驱逐，需要保护
	 * 或者当前refs已经饱和(4)，也就是tier是最大值了，也需要保护？
	 */
	if (tier > tier_idx || refs == BIT(LRU_REFS_WIDTH)) {
		int hist = lru_hist_from_seq(lrugen->min_seq[type]);

		/* 将页面提升一代，并且加入到对应的MGLRU链表（前面需要拿LRU锁） */
		gen = folio_inc_gen(lruvec, folio, false);
		list_move_tail(&folio->lru, &lrugen->folios[gen][type][zone]);

		WRITE_ONCE(lrugen->protected[hist][type][tier - 1],
			   lrugen->protected[hist][type][tier - 1] + delta);
		return true;
	}

	/* ineligible */
	/* ineligible：zone 不符合或 PG_lru 已清除 */
	if (!folio_test_lru(folio) || zone > sc->reclaim_idx) {
		/* 提升一代，暂缓驱逐 */
		gen = folio_inc_gen(lruvec, folio, false);
		list_move_tail(&folio->lru, &lrugen->folios[gen][type][zone]);
		return true;
	}

	/* waiting for writeback */
	/* 正在回写，不能驱逐 */
	if (folio_test_locked(folio) || folio_test_writeback(folio) ||
	    (type == LRU_GEN_FILE && folio_test_dirty(folio))) {
		gen = folio_inc_gen(lruvec, folio, true);
		/* 注意，这里和上面不同，是移动到头部(较热端) */
		list_move(&folio->lru, &lrugen->folios[gen][type][zone]);
		return true;
	}
	/*
	 * 三种情况对应不同的 reclaiming 和 list 位置：
	 * protection/ineligible: folio_inc_gen(false) + list_move_tail → 加入新代链表尾部（较冷端）
	 * writeback waiting:     folio_inc_gen(true)  + list_move      → 加入新代链表头部（较热端）
	 */

	return false;
}

static bool isolate_folio(struct lruvec *lruvec, struct folio *folio, struct scan_control *sc)
{
	bool success;

	/* swap constrained(约束) */
	if (!(sc->gfp_mask & __GFP_IO) &&
	    (folio_test_dirty(folio) ||
	     (folio_test_anon(folio) && !folio_test_swapcache(folio))))
		return false;

	/* raced with release_pages() */
	if (!folio_try_get(folio))
		return false;

	/* raced with another isolation */
	/* 已经被隔离了 */
	if (!folio_test_clear_lru(folio)) {
		folio_put(folio);
		return false;
	}

	/* see the comment on MAX_NR_TIERS */
	/*
        * 若 PG_referenced 未被置位，说明此 folio 近期没有被访问过，
        * 将 LRU_REFS_MASK（引用计数位域）和 LRU_REFS_FLAGS（辅助标志）一并清零，
        * 即把 tier 重置为 0。
        * 原因：folio 即将被发送到 shrink_folio_list() 尝试驱逐，
        * tier 信息已无意义；清零可确保下次重新加入 LRU 时从 tier 0 起算。
	*/
	if (!folio_test_referenced(folio))
		set_mask_bits(&folio->flags, LRU_REFS_MASK | LRU_REFS_FLAGS, 0);

	/* for shrink_folio_list() */
        /*
         * 清除 PG_reclaim：该 bit 在 sort_folio() 对等待回写的 folio 调用
         * folio_inc_gen(reclaiming=true) 时被置位，表示"正在排队等回写"。
         * 现在进入正式隔离流程，清除此标志，给 shrink_folio_list() 一个干净状态。
         */
	folio_clear_reclaim(folio);
        /*
         * 清除 PG_referenced：shrink_folio_list() 依赖此 bit 决定是否保留 folio，
         * 在隔离时清零，确保判断基于隔离后的新鲜访问记录，而非历史遗留。
         */
	folio_clear_referenced(folio);

	/* 将页面从mglru链表中移除 */
	success = lru_gen_del_folio(lruvec, folio, true);
	VM_WARN_ON_ONCE_FOLIO(!success, folio);

	return true;
}

static int scan_folios(struct lruvec *lruvec, struct scan_control *sc,
		       int type, int tier, struct list_head *list)
{
	int i;
	int gen;
	enum vm_event_item item;
	int sorted = 0;
	int scanned = 0;
	int isolated = 0;
	int skipped = 0;
	int remaining = MAX_LRU_BATCH;
	struct lru_gen_folio *lrugen = &lruvec->lrugen;
	struct mem_cgroup *memcg = lruvec_memcg(lruvec);

	VM_WARN_ON_ONCE(!list_empty(list));

	if (get_nr_gens(lruvec, type) == MIN_NR_GENS)
		return 0;

	/* 取最老的gen */
	gen = lru_gen_from_seq(lrugen->min_seq[type]);

	for (i = MAX_NR_ZONES; i > 0; i--) {
		LIST_HEAD(moved);
		int skipped_zone = 0;
		int zone = (sc->reclaim_idx + i) % MAX_NR_ZONES;
		struct list_head *head = &lrugen->folios[gen][type][zone];

		/*
		 * 遍历所有合适的zone
		 * 对 &lrugen->folios[gen][type][zone]的所有页面调用sort_folio()进行筛选
		 * 如果合适，则调用isolate_folio()隔离出来，如果隔离失败，则放回原来的链表
		 */
		while (!list_empty(head)) {
			struct folio *folio = lru_to_folio(head);
			int delta = folio_nr_pages(folio);

			VM_WARN_ON_ONCE_FOLIO(folio_test_unevictable(folio), folio);
			VM_WARN_ON_ONCE_FOLIO(folio_test_active(folio), folio);
			VM_WARN_ON_ONCE_FOLIO(folio_is_file_lru(folio) != type, folio);
			VM_WARN_ON_ONCE_FOLIO(folio_zonenum(folio) != zone, folio);

			scanned += delta;

			/*
			 * 在扫描的时候先判断页面是否可以被隔离回收，如果不可以，则将其动态提升到更新的代
			 * 如果可以被隔离回收，则进入下面的isolate_folio
			 */
			if (sort_folio(lruvec, folio, sc, tier))
				sorted += delta;
			else if (isolate_folio(lruvec, folio, sc)) {
				/* 将页面加入list */
				list_add(&folio->lru, list);
				isolated += delta;
			} else {
				/* 将跳过的页面从lru链表删除并加入moved链表  */
				list_move(&folio->lru, &moved);
				skipped_zone += delta;
			}

			if (!--remaining || max(isolated, skipped_zone) >= MIN_LRU_BATCH)
				break;
		}

		if (skipped_zone) {
			/* 将跳过的页面重新加到原来的链表 */
			list_splice(&moved, head);
			__count_zid_vm_events(PGSCAN_SKIP, zone, skipped_zone);
			skipped += skipped_zone;
		}

		if (!remaining || isolated >= MIN_LRU_BATCH)
			break;
	}

	item = PGSCAN_KSWAPD + reclaimer_offset();
	if (!cgroup_reclaim(sc)) {
		__count_vm_events(item, isolated);
		__count_vm_events(PGREFILL, sorted);
	}
	__count_memcg_events(memcg, item, isolated);
	__count_memcg_events(memcg, PGREFILL, sorted);
	__count_vm_events(PGSCAN_ANON + type, isolated);
	trace_mm_vmscan_lru_isolate(sc->reclaim_idx, sc->order, MAX_LRU_BATCH,
				scanned, skipped, isolated,
				type ? LRU_INACTIVE_FILE : LRU_INACTIVE_ANON);

	/*
	 * There might not be eligible folios due to reclaim_idx. Check the
	 * remaining to prevent livelock if it's not making progress.
	 */
       /*
        * 返回值语义：
        * - `isolated || !remaining`（有隔离成果 或 配额耗尽）→ 返回 scanned，
        *   告知调用者扫描了多少页，用于推进 min_seq 和统计。
	*
        * - 否则（两者都不满足：既没隔离到页面，配额也没耗尽）→ 返回 0，
        *   ** 表示本 type 当前 zone 内没有合适页面（可能全被 sort/skip）**，
        *   调用者（isolate_folios）将尝试切换到另外一种type。
        *
        * NOTE：即使 reclaim_idx 限制导致没有合适 folio，remaining 减少也能防止活锁。
        */
	return isolated || !remaining ? scanned : 0;
}

/*
 * 获取可以驱逐的最大tier
 *
 * 判断条件是从tier1开始，如果tierN的refault率远远大于tier0，认为tierN是不可以被驱逐的
 * 所以返回上一个tier，也就是tier(N-1)，所以从tier0到tier(N-1)都是可以被驱逐的
 *
 * 如果refault率判断条件都不成立，则至二级返回最大的tier，也就是3
 */
static int get_tier_idx(struct lruvec *lruvec, int type)
{
	int tier;
	struct ctrl_pos sp, pv;

	/*
	 * To leave a margin for fluctuations, use a larger gain factor (1:2).
	 * This value is chosen because any other tier would have at least twice
	 * as many refaults as the first tier.
	 */
	read_ctrl_pos(lruvec, type, 0, 1, &sp);
	for (tier = 1; tier < MAX_NR_TIERS; tier++) {
		read_ctrl_pos(lruvec, type, tier, 2, &pv);
		/*
		 * !(pv * 1 <= sp * 2)
		 * 当前tier的refault率 高于 tier0 refault率的两倍
		 * 也就是当前tier的refault率 远高于 tier0的refault率
		 * 具体看positive_ctrl_err的解释
		 *
		 * 条件成立break出去说明tierN的refault率远远大于tier0的refault率
		 *
		 * tierN的refault率太高，说明被回收后很快又被访问了，所以这个tier不能被驱逐(代价太高)
		 */
		if (!positive_ctrl_err(&sp, &pv))
			break;
	}

	/* 返回上一个tier，可以被驱逐的最高tier */
	return tier - 1;
}

/* 待分析 */
static int get_type_to_scan(struct lruvec *lruvec, int swappiness, int *tier_idx)
{
	int type, tier;
	struct ctrl_pos sp, pv;
	int gain[ANON_AND_FILE] = { swappiness, MAX_SWAPPINESS - swappiness };

	/*
	 * Compare the first tier of anon with that of file to determine which
	 * type to scan. Also need to compare other tiers of the selected type
	 * with the first tier of the other type to determine the last tier (of
	 * the selected type) to evict.
	 */
	read_ctrl_pos(lruvec, LRU_GEN_ANON, 0, gain[LRU_GEN_ANON], &sp);
	read_ctrl_pos(lruvec, LRU_GEN_FILE, 0, gain[LRU_GEN_FILE], &pv);
	type = positive_ctrl_err(&sp, &pv);

	read_ctrl_pos(lruvec, !type, 0, gain[!type], &sp);
	for (tier = 1; tier < MAX_NR_TIERS; tier++) {
		read_ctrl_pos(lruvec, type, tier, gain[type], &pv);
		if (!positive_ctrl_err(&sp, &pv))
			break;
	}

	*tier_idx = tier - 1;

	return type;
}

static int isolate_folios(struct lruvec *lruvec, struct scan_control *sc, int swappiness,
			  int *type_scanned, struct list_head *list)
{
	int i;
	int type;
	int scanned;
	int tier = -1;
	/* 获取min_seq[LRU_GEN_ANON] 和 min_seq[LRU_GEN_FILE] */
	DEFINE_MIN_SEQ(lruvec);

	/*
	 * Try to make the obvious choice first, and if anon and file are both
	 * available from the same generation,
	 * 1. Interpret swappiness 1 as file first and MAX_SWAPPINESS as anon
	 *    first.
	 * 2. If !__GFP_IO, file first since clean pagecache is more likely to
	 *    exist than clean swapcache.
	 */
	/*
	 * 通过swappiness和min_seq判断应该回收哪一类页面
	 */
	if (!swappiness)
		type = LRU_GEN_FILE;
	else if (min_seq[LRU_GEN_ANON] < min_seq[LRU_GEN_FILE])
		type = LRU_GEN_ANON;
	else if (swappiness == 1)
		type = LRU_GEN_FILE;
	else if (swappiness == MAX_SWAPPINESS)
		type = LRU_GEN_ANON;
	else if (!(sc->gfp_mask & __GFP_IO))
		type = LRU_GEN_FILE;
	else
		type = get_type_to_scan(lruvec, swappiness, &tier);

	/*
	 * LRU_GEN_ANON = 0 , LRU_GEN_FILE = 1
	 * #define ANON_AND_FILE 2
	 *
	 * 根据type，从ANON开始遍历: type=FILE，只回收FILE，type=ANON，先回收ANON再回收FILE
	 */
	for (i = !swappiness; i < ANON_AND_FILE; i++) {
		if (tier < 0)
			tier = get_tier_idx(lruvec, type);

		/*
		 * 如果没成功隔离页面并且扫描配额用完，
		 * 说明当前type&zone内没有合适的页面（可能全被sort/skip）
		 * 尝试切换到另一种type
		 */
		scanned = scan_folios(lruvec, sc, type, tier, list);
		if (scanned)
			break;

		type = !type;
		tier = -1;
	}

	*type_scanned = type;

	return scanned;
}

static int evict_folios(struct lruvec *lruvec, struct scan_control *sc, int swappiness)
{
	int type;
	int scanned;
	int reclaimed;
	LIST_HEAD(list);
	LIST_HEAD(clean);
	struct folio *folio;
	struct folio *next;
	enum vm_event_item item;
	struct reclaim_stat stat;
	struct lru_gen_mm_walk *walk;
	bool skip_retry = false;
	struct mem_cgroup *memcg = lruvec_memcg(lruvec);
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);

	spin_lock_irq(&lruvec->lru_lock);

	scanned = isolate_folios(lruvec, sc, swappiness, &type, &list);

	/*
	 * 尝试推进最老的gen
	 * 如果最老gen的页面都被隔离出来了，则可以被推进
	 */
	scanned += try_to_inc_min_seq(lruvec, swappiness);

	if (get_nr_gens(lruvec, !swappiness) == MIN_NR_GENS)
		scanned = 0;

	spin_unlock_irq(&lruvec->lru_lock);

	if (list_empty(&list))
		return scanned;
retry:
	reclaimed = shrink_folio_list(&list, pgdat, sc, &stat, false);
	sc->nr_reclaimed += reclaimed;
	trace_mm_vmscan_lru_shrink_inactive(pgdat->node_id,
			scanned, reclaimed, &stat, sc->priority,
			type ? LRU_INACTIVE_FILE : LRU_INACTIVE_ANON);

	list_for_each_entry_safe_reverse(folio, next, &list, lru) {
		if (!folio_evictable(folio)) {
			list_del(&folio->lru);
			folio_putback_lru(folio);
			continue;
		}

		if (folio_test_reclaim(folio) &&
		    (folio_test_dirty(folio) || folio_test_writeback(folio))) {
			/* restore LRU_REFS_FLAGS cleared by isolate_folio() */
			if (folio_test_workingset(folio))
				folio_set_referenced(folio);
			continue;
		}

		if (skip_retry || folio_test_active(folio) || folio_test_referenced(folio) ||
		    folio_mapped(folio) || folio_test_locked(folio) ||
		    folio_test_dirty(folio) || folio_test_writeback(folio)) {
			/* don't add rejected folios to the oldest generation */
			set_mask_bits(&folio->flags, LRU_REFS_MASK | LRU_REFS_FLAGS,
				      BIT(PG_active));
			continue;
		}

		/* retry folios that may have missed folio_rotate_reclaimable() */
		list_move(&folio->lru, &clean);
	}

	spin_lock_irq(&lruvec->lru_lock);

	move_folios_to_lru(lruvec, &list);

	walk = current->reclaim_state->mm_walk;
	if (walk && walk->batched) {
		walk->lruvec = lruvec;
		reset_batch_size(walk);
	}

	item = PGSTEAL_KSWAPD + reclaimer_offset();
	if (!cgroup_reclaim(sc))
		__count_vm_events(item, reclaimed);
	__count_memcg_events(memcg, item, reclaimed);
	__count_vm_events(PGSTEAL_ANON + type, reclaimed);

	spin_unlock_irq(&lruvec->lru_lock);

	list_splice_init(&clean, &list);

	if (!list_empty(&list)) {
		skip_retry = true;
		goto retry;
	}

	return scanned;
}

/*
 * 在扫描页面的时候(gen_nr_to_scan)，判断是否需要老化
 *
 * 1.gen <= 1，需要老化’
 * 2.gen == 4，不需要老化；
 * 3.gen == 3:
 *	3.1 热页过多（超过1/2），需要老化
 *	3.2 冷页过少（超过1/4），需要老化
 */
static bool should_run_aging(struct lruvec *lruvec, unsigned long max_seq,
			     bool can_swap, unsigned long *nr_to_scan)
{
	int gen, type, zone;
	unsigned long old = 0;
	unsigned long young = 0;
	unsigned long total = 0;
	struct lru_gen_folio *lrugen = &lruvec->lrugen;
	DEFINE_MIN_SEQ(lruvec);

	/* whether this lruvec is completely out of cold folios */
	/*
	 * MIN_NR_GENS = 2
	 * 如果当前类型（ANON/FILE）的gen太少，只有一个或两个gen
	 * 那说明老的页面太少了，必须马上老化
	 */
	if (min_seq[!can_swap] + MIN_NR_GENS > max_seq) {
		*nr_to_scan = 0;
		return true;
	}

	for (type = !can_swap; type < ANON_AND_FILE; type++) {
		unsigned long seq;

		for (seq = min_seq[type]; seq <= max_seq; seq++) {
			unsigned long size = 0;

			gen = lru_gen_from_seq(seq);

			for (zone = 0; zone < MAX_NR_ZONES; zone++)
				size += max(READ_ONCE(lrugen->nr_pages[gen][type][zone]), 0L);

			/* 获取所有gen的总大小 */
			total += size;
			/* 获取最新gen的总大小 */
			if (seq == max_seq)
				young += size;
			/* 获取最老gen的总大小, 只有在存在4个gen的情况下才统计 */
			else if (seq + MIN_NR_GENS == max_seq)
				old += size;
		}
	}

	*nr_to_scan = total;

	/*
	 * The aging tries to be lazy to reduce the overhead, while the eviction
	 * stalls when the number of generations reaches MIN_NR_GENS. Hence, the
	 * ideal number of generations is MIN_NR_GENS+1.
	 */
	/*
	 * 三个gen是最理想的老化状态
	 * 如果有4个gen，则不老化
	 *
	 * question：这个判断能否提前，这样还不需要走for循环的size计算
	 * answer: 不行，因为返回后还需要用到nr_to_scan
	 * 当 gen=4、should_run_aging 返回 false 时，调用者get_nr_to_scan()依然需要通过 nr_to_scan的值
	 * 知道这个 lruvec 里一共有多少页面可供回收（用于计算本次 eviction 的扫描配额 nr_to_scan >> sc->priority）。
	 */
	if (min_seq[!can_swap] + MIN_NR_GENS < max_seq)
		return false;

	/*
	 * It's also ideal to spread pages out evenly, i.e., 1/(MIN_NR_GENS+1)
	 * of the total number of pages for each generation. A reasonable range
	 * for this average portion is [1/MIN_NR_GENS, 1/(MIN_NR_GENS+2)]. The
	 * aging cares about the upper bound of hot pages, while the eviction
	 * cares about the lower bound of cold pages.
	 */
	/* gen=3 的情况 */
	/* 如果热页过多(young > total/2)，则需要老化 */
	if (young * MIN_NR_GENS > total)
		return true;
	/* 如果冷页过少(old < total/4)，则需要老化 */
	if (old * (MIN_NR_GENS + 2) < total)
		return true;

	return false;
}

/*
 * For future optimizations:
 * 1. Defer(延迟) try_to_inc_max_seq() to workqueues to reduce latency for memcg
 *    reclaim.
 */
/*
 * 扫描目标lruvec的MGLRU链表，判断是否需要老化，需要老化则推进gen
 *
 * 返回值：
 * 返回-1：说明老化成功，刚老化完成，需要让新的gen稳定，告诉调用者以先不回收
 * 返回 0：说明老化失败，这时候应该没有足够多的冷页来回收，告诉调用者先不回收
 * 返回>0：说明存在可以回收的页面
 */
static long get_nr_to_scan(struct lruvec *lruvec, struct scan_control *sc, bool can_swap)
{
	bool success;
	unsigned long nr_to_scan;
	/* 从lruve中获取对应的memcg */
	struct mem_cgroup *memcg = lruvec_memcg(lruvec);

	/*
	 * 定义并获取max_seq
	 * 相当于unsigned long max_seq = READ_ONCE((lruvec)->lrugen.max_seq)
	 */
	DEFINE_MAX_SEQ(lruvec);

	if (mem_cgroup_below_min(sc->target_mem_cgroup, memcg))
		return -1;

	/*
	 * 判断是否需要进行页面老化，获取nr_to_scan
	 */
	success = should_run_aging(lruvec, max_seq, can_swap, &nr_to_scan);

	/* try to scrape(清除) all its memory if this memcg was deleted */
	/*
	 * nr_to_scan不为0，说明gen的个数 >= 3
	 *
	 * mem_cgroup_online(memcg) 返回 false，
	 * 说明这个 memcg 已经被删除（进程退出或 cgroup 被 rmdir），正处于离线/回收状态。
	 *
	 * 此时的策略是：不管 aging 是否需要，直接返回全部页面数量，一次性把这个 memcg 的内存全部回收掉。
	 *
	 * 返回全量nr_to_scan，而不做>> sc->priority缩放，意味着不限速、全力回收
	 * nr_to_scan && 是防御性检查，确保memcg确实还有页面
	 */
	if (nr_to_scan && !mem_cgroup_online(memcg))
		return nr_to_scan;

	/* try to get away with not aging at the default priority */
	/*
	 * 1. !success，不需要老化，返回扫描配额
	 * 2. sc->priority == DEF_PRIORITY，回收压力等级是默认值12，也就是回收压力最小，也不需要老化
	 *
	 * 注释已说明：在默认优先级下尽量避免 aging 开销
	 * 这是 lazy aging 的又一层体现：只有在内存压力升高（priority 降低）时才真正去执行 aging。
	 */
	if (!success || sc->priority == DEF_PRIORITY)
		return nr_to_scan >> sc->priority;

	/* stop scanning this lruvec as it's low on cold folios */
	/*
	 * 可执行老化的条件：
	 *	1. success = true（需要 aging）
	 *	2. 同时sc->priority < DEF_PRIORITY（内存压力已经升高）
	 *
	 * 返回值说明：
	 *	1. -1: aging 成功（max_seq 推进）,告知调用者跳过这个 lruvec 本轮 eviction（刚 aging 完，让新 gen 稳定一下）
	 *	2.  0: aging 失败（有其他线程已经推进了 max_seq），已经没有页面可以老化了
	 */
	return try_to_inc_max_seq(lruvec, max_seq, can_swap, false) ? -1 : 0;
}

static bool should_abort_scan(struct lruvec *lruvec, struct scan_control *sc)
{
	int i;
	enum zone_watermarks mark;

	/* don't abort memcg reclaim to ensure fairness */
	if (!root_reclaim(sc))
		return false;

	if (sc->nr_reclaimed >= max(sc->nr_to_reclaim, compact_gap(sc->order)))
		return true;

	/* check the order to exclude compaction-induced reclaim */
	if (!current_is_kswapd() || sc->order)
		return false;

	mark = sysctl_numa_balancing_mode & NUMA_BALANCING_MEMORY_TIERING ?
	       WMARK_PROMO : WMARK_HIGH;

	for (i = 0; i <= sc->reclaim_idx; i++) {
		struct zone *zone = lruvec_pgdat(lruvec)->node_zones + i;
		unsigned long size = wmark_pages(zone, mark) + MIN_LRU_BATCH;

		if (managed_zone(zone) && !zone_watermark_ok(zone, 0, size, sc->reclaim_idx, 0))
			return false;
	}

	/* kswapd should abort if all eligible zones are safe */
	return true;
}

static bool try_to_shrink_lruvec(struct lruvec *lruvec, struct scan_control *sc)
{
	long nr_to_scan;
	unsigned long scanned = 0;
	int swappiness = get_swappiness(lruvec, sc);

	while (true) {
		int delta;

		/* 获取扫描配额 */
		nr_to_scan = get_nr_to_scan(lruvec, sc, swappiness);
		/*
		 * 返回-1：说明老化成功，刚老化完成，需要让新的gen稳定，所以先不回收
		 * 返回 0：说明老化失败，没有冷页可回收，所以也先不回收
		 */
		if (nr_to_scan <= 0)
			break;

		/* 页面回收核心，返回处理过的页面数 */
		delta = evict_folios(lruvec, sc, swappiness);
		/* eviction没有任何进展，退出 */
		if (!delta)
			break;

		scanned += delta;
		/* 如果处理过的页面数量已经等于或者超过扫描配额，则退出 */
		if (scanned >= nr_to_scan)
			break;

		/* 判断回收目标是否已经达成，达成则退出 */
		if (should_abort_scan(lruvec, sc))
			break;

		/* 主动让出cpu，避免长时间占用 */
		cond_resched();
	}

	/* whether this lruvec should be rotated */
	/*
	 * 返回值
	 *	true：最后一次 get_nr_to_scan 返回 -1，说明本次循环是因为触发了 aging 而退出的。调用者据此将这个 lruvec 对应的 memcg 旋转到 "young" 位置（回收完成，给予奖励）。
	 *	false：正常耗尽扫描配额或无法继续，调用者根据其他条件决定 memcg 的位置。
	 */
	return nr_to_scan < 0;
}

static int shrink_one(struct lruvec *lruvec, struct scan_control *sc)
{
	bool success;
	unsigned long scanned = sc->nr_scanned;
	unsigned long reclaimed = sc->nr_reclaimed;
	struct mem_cgroup *memcg = lruvec_memcg(lruvec);
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);

	/* lru_gen_age_node() called mem_cgroup_calculate_protection() */
	/*
	* memcg 当前内存用量低于 memory.min 保护阈值，完全不需要对该memcg进行回收。
	* 直接返回 MEMCG_LRU_YOUNG，让这个 memcg 在 memcg LRU 中旋转到最年轻的位置，
	* 使其短期内不再被回收选中。
	*/
	if (mem_cgroup_below_min(NULL, memcg))
		return MEMCG_LRU_YOUNG;

	/*
	 * memcg 用量低于 memory.low 保护阈值，应尽量不回收。这里有一次缓冲机制：
	 * - 若 memcg 当前 seg 不是 MEMCG_LRU_TAIL：返回 MEMCG_LRU_TAIL，让其在当前 gen 的尾部等待，给一次额外的机会
	 * - 若 memcg 已经是 MEMCG_LRU_TAIL（第二次机会也用完了）：无奈允许回收，触发 MEMCG_LOW 事件通知
	 */
	if (mem_cgroup_below_low(NULL, memcg)) {
		/* see the comment on MEMCG_NR_GENS */
		if (READ_ONCE(lruvec->lrugen.seg) != MEMCG_LRU_TAIL)
			return MEMCG_LRU_TAIL;

		memcg_memory_event(memcg, MEMCG_LOW);
	}

	/* MGLRU页面回收主逻辑 */
	success = try_to_shrink_lruvec(lruvec, sc);

	shrink_slab(sc->gfp_mask, pgdat->node_id, memcg, sc->priority);

	if (!sc->proactive)
		vmpressure(sc->gfp_mask, memcg, false, sc->nr_scanned - scanned,
			   sc->nr_reclaimed - reclaimed);

	flush_reclaim_state(sc);

	if (success && mem_cgroup_online(memcg))
		return MEMCG_LRU_YOUNG;

	if (!success && lruvec_is_sizable(lruvec, sc))
		return 0;

	/* one retry if offlined or too small */
	return READ_ONCE(lruvec->lrugen.seg) != MEMCG_LRU_TAIL ?
	       MEMCG_LRU_TAIL : MEMCG_LRU_YOUNG;
}

/*
 * shrink_many - 全局回收路径下对多个 memcg 进行页面回收
 *
 * 参数：
 *   @pgdat: 当前 NUMA node 的数据结构，包含 memcg_lru（memcg 的两代 FIFO）
 *   @sc:    回收控制参数（目标回收量、优先级、gfp 等）
 *
 * 返回值：无
 *
 * 调用场景：lru_gen_shrink_node() 在启用 memcg 时调用本函数，
 * 替代传统的 mem_cgroup_iter() 树遍历，以 memcg LRU 的方式
 * 有优先级地选取 memcg 进行回收。
 */
static void shrink_many(struct pglist_data *pgdat, struct scan_control *sc)
{
	int op;
	int gen;
	int bin;
	int first_bin;
	struct lruvec *lruvec;
	struct lru_gen_folio *lrugen;
	struct mem_cgroup *memcg;
	struct hlist_nulls_node *pos;

	gen = get_memcg_gen(READ_ONCE(pgdat->memcg_lru.seq));
	bin = first_bin = get_random_u32_below(MEMCG_NR_BINS);
restart:
	op = 0;
	memcg = NULL;

	rcu_read_lock();
	/*
         * 遍历 fifo[gen][bin]链表头所对应的链表的所有 lrugen 节点。
         *
         * hlist_nulls_for_each_entry_rcu 展开逻辑：
         *   pos  = rcu_dereference(fifo[gen][bin].first)   // 取链表头
         *   while (!is_a_nulls(pos)):                       // 遇到 nulls 标记则停止
         *       lrugen = container_of(pos, lru_gen_folio, list)
         *       pos    = rcu_dereference(pos->next)         // 步进到下一个节点
         *
         * 循环结束后 pos 指向 nulls 标记指针（编码了 gen 值），
         * 或者因 break 退出时 pos 指向某个普通节点。
         */
	hlist_nulls_for_each_entry_rcu(lrugen, pos, &pgdat->memcg_lru.fifo[gen][bin], list) {
		if (op) {
			lru_gen_rotate_memcg(lruvec, op);
			op = 0;
		}

		mem_cgroup_put(memcg);
		memcg = NULL;

		if (gen != READ_ONCE(lrugen->gen))
			continue;

		lruvec = container_of(lrugen, struct lruvec, lrugen);
		memcg = lruvec_memcg(lruvec);

		/*
                 * 尝试获取 memcg 的引用计数（tryget 不会阻塞）：
                 *   如果 memcg 正在 offline（引用计数已降为 0），tryget 返回 false。
                 *   此时调用 lru_gen_release_memcg() 将其从 fifo 链表中摘除，
                 *   并跳过本次回收。
                 *
                 * 持有引用计数的目的：防止在 rcu_read_unlock() 之后、
                 * shrink_one() 执行期间 memcg 被释放。
                 */
		if (!mem_cgroup_tryget(memcg)) {
			lru_gen_release_memcg(memcg);
			memcg = NULL;
			continue;
		}

		rcu_read_unlock();

		/*
                 * 对当前 memcg 的 lruvec 执行实际的页面回收：
                 *   - 检查 memory.min/low 保护，决定是否跳过或缓冲；
                 *   - 调用 try_to_shrink_lruvec() 执行 MGLRU 页面回收；
                 *   - 调用 shrink_slab() 回收 slab 对象；
                 *   - 返回值是该 memcg 下一步应执行的 rotate 操作：
                 *       MEMCG_LRU_YOUNG：回收成功或受保护，移入 young 代
                 *       MEMCG_LRU_TAIL ：给一次缓冲机会，移到当前代尾部
                 *       0              ：继续留在 old 代当前位置
                 *
                 * 返回值存入 op，在下一轮循环开头（或循环结束后）执行 rotate。
                 */
		op = shrink_one(lruvec, sc);

		rcu_read_lock();

                /*
                 * 检查是否应该提前终止本次全局回收：
                 *   - 已回收页面数达到目标（nr_reclaimed >= nr_to_reclaim）；
                 *   - kswapd 场景下所有 zone 水位已恢复到 WMARK_HIGH 以上。
                 * 满足条件则 break，pos 此时指向某个普通节点（非 nulls）。
                 */
		if (should_abort_scan(lruvec, sc))
			break;
	}

	rcu_read_unlock();

	if (op)
		lru_gen_rotate_memcg(lruvec, op);

	mem_cgroup_put(memcg);

        /*
         * 判断循环退出原因：
         *   - 若 pos 不是 nulls 标记（is_a_nulls 为 false），说明是被
         *     should_abort_scan() 触发 break 提前退出的，回收目标已达成，
         *     直接返回，不再遍历其他 bin。
         *   - 若 pos 是 nulls 标记，说明当前 bin 正常遍历完毕，
         *     继续后续的 restart 检测和 bin 轮转逻辑。
         */
	if (!is_a_nulls(pos))
		return;

	/* restart if raced with lru_gen_rotate_memcg() */
	/*
	 * nulls 标记竞态检测（restart 判断）：
	 *
	 *   fifo[i][j] 初始化时：INIT_HLIST_NULLS_HEAD(&fifo[i][j], i)
	 *   即 fifo[i][j].first = NULLS_MARKER(i)，nulls 标记编码了 gen=i。
	 *
	 *   正常情况：遍历 fifo[gen][bin] 到链表尾，pos = NULLS_MARKER(gen)，
	 *   get_nulls_value(pos) == gen，检测通过。
	 *
	 *   异常情况：遍历过程中某个节点被 lru_gen_rotate_memcg() 从
	 *   fifo[gen][bin_A] 移到 fifo[other_gen][bin_B]，遍历的 next 指针
	 *   跟着跑到了 fifo[other_gen][bin_B] 的链表尾，读到的 nulls 值是
	 *   other_gen 而不是 gen，说明本次遍历可能漏掉了部分 memcg，
	 *   需要 goto restart 重新从当前 bin 开始遍历。
	 */
	if (gen != get_nulls_value(pos))
		goto restart;

	/* try the rest of the bins of the current generation */
        /*
         * 当前 bin 遍历完毕且无竞态，轮转到下一个 bin：
         *   get_memcg_bin(bin+1) = (bin+1) % MEMCG_NR_BINS
         *   若还没转完一圈（bin != first_bin），goto restart 继续遍历下一个 bin；
         *   若已转完一圈（bin == first_bin），退出函数，本次全局回收结束。
         */
	bin = get_memcg_bin(bin + 1);
	if (bin != first_bin)
		goto restart;
}

static void lru_gen_shrink_lruvec(struct lruvec *lruvec, struct scan_control *sc)
{
	struct blk_plug plug;

	VM_WARN_ON_ONCE(root_reclaim(sc));
	VM_WARN_ON_ONCE(!sc->may_writepage || !sc->may_unmap);

	lru_add_drain();

	blk_start_plug(&plug);

	set_mm_walk(NULL, sc->proactive);

	if (try_to_shrink_lruvec(lruvec, sc))
		lru_gen_rotate_memcg(lruvec, MEMCG_LRU_YOUNG);

	clear_mm_walk();

	blk_finish_plug(&plug);
}

static void lru_gen_shrink_node(struct pglist_data *pgdat, struct scan_control *sc)
{
	struct blk_plug plug;
	unsigned long reclaimed = sc->nr_reclaimed;

	VM_WARN_ON_ONCE(!root_reclaim(sc));

	/*
	 * Unmapped clean folios are already prioritized. Scanning for more of
	 * them is likely futile and can cause high reclaim latency when there
	 * is a large number of memcgs.
	 */
	if (!sc->may_writepage || !sc->may_unmap)
		goto done;

	lru_add_drain();

	blk_start_plug(&plug);

	set_mm_walk(pgdat, sc->proactive);

	set_initial_priority(pgdat, sc);

	if (current_is_kswapd())
		sc->nr_reclaimed = 0;

	if (mem_cgroup_disabled())
		shrink_one(&pgdat->__lruvec, sc);
	else
		shrink_many(pgdat, sc);

	if (current_is_kswapd())
		sc->nr_reclaimed += reclaimed;

	clear_mm_walk();

	blk_finish_plug(&plug);
done:
	if (sc->nr_reclaimed > reclaimed)
		pgdat->kswapd_failures = 0;
}

/******************************************************************************
 *                          state change
 ******************************************************************************/

static bool __maybe_unused state_is_valid(struct lruvec *lruvec)
{
	struct lru_gen_folio *lrugen = &lruvec->lrugen;

	if (lrugen->enabled) {
		enum lru_list lru;

		for_each_evictable_lru(lru) {
			if (!list_empty(&lruvec->lists[lru]))
				return false;
		}
	} else {
		int gen, type, zone;

		for_each_gen_type_zone(gen, type, zone) {
			if (!list_empty(&lrugen->folios[gen][type][zone]))
				return false;
		}
	}

	return true;
}

static bool fill_evictable(struct lruvec *lruvec)
{
	enum lru_list lru;
	int remaining = MAX_LRU_BATCH;

	for_each_evictable_lru(lru) {
		int type = is_file_lru(lru);
		bool active = is_active_lru(lru);
		struct list_head *head = &lruvec->lists[lru];

		while (!list_empty(head)) {
			bool success;
			struct folio *folio = lru_to_folio(head);

			VM_WARN_ON_ONCE_FOLIO(folio_test_unevictable(folio), folio);
			VM_WARN_ON_ONCE_FOLIO(folio_test_active(folio) != active, folio);
			VM_WARN_ON_ONCE_FOLIO(folio_is_file_lru(folio) != type, folio);
			VM_WARN_ON_ONCE_FOLIO(folio_lru_gen(folio) != -1, folio);

			lruvec_del_folio(lruvec, folio);
			success = lru_gen_add_folio(lruvec, folio, false);
			VM_WARN_ON_ONCE(!success);

			if (!--remaining)
				return false;
		}
	}

	return true;
}

static bool drain_evictable(struct lruvec *lruvec)
{
	int gen, type, zone;
	int remaining = MAX_LRU_BATCH;

	for_each_gen_type_zone(gen, type, zone) {
		struct list_head *head = &lruvec->lrugen.folios[gen][type][zone];

		while (!list_empty(head)) {
			bool success;
			struct folio *folio = lru_to_folio(head);

			VM_WARN_ON_ONCE_FOLIO(folio_test_unevictable(folio), folio);
			VM_WARN_ON_ONCE_FOLIO(folio_test_active(folio), folio);
			VM_WARN_ON_ONCE_FOLIO(folio_is_file_lru(folio) != type, folio);
			VM_WARN_ON_ONCE_FOLIO(folio_zonenum(folio) != zone, folio);

			success = lru_gen_del_folio(lruvec, folio, false);
			VM_WARN_ON_ONCE(!success);
			lruvec_add_folio(lruvec, folio);

			if (!--remaining)
				return false;
		}
	}

	return true;
}

static void lru_gen_change_state(bool enabled)
{
	static DEFINE_MUTEX(state_mutex);

	struct mem_cgroup *memcg;

	cgroup_lock();
	cpus_read_lock();
	get_online_mems();
	mutex_lock(&state_mutex);

	if (enabled == lru_gen_enabled())
		goto unlock;

	if (enabled)
		static_branch_enable_cpuslocked(&lru_gen_caps[LRU_GEN_CORE]);
	else
		static_branch_disable_cpuslocked(&lru_gen_caps[LRU_GEN_CORE]);

	memcg = mem_cgroup_iter(NULL, NULL, NULL);
	do {
		int nid;

		for_each_node(nid) {
			struct lruvec *lruvec = get_lruvec(memcg, nid);

			spin_lock_irq(&lruvec->lru_lock);

			VM_WARN_ON_ONCE(!seq_is_valid(lruvec));
			VM_WARN_ON_ONCE(!state_is_valid(lruvec));

			lruvec->lrugen.enabled = enabled;

			while (!(enabled ? fill_evictable(lruvec) : drain_evictable(lruvec))) {
				spin_unlock_irq(&lruvec->lru_lock);
				cond_resched();
				spin_lock_irq(&lruvec->lru_lock);
			}

			spin_unlock_irq(&lruvec->lru_lock);
		}

		cond_resched();
	} while ((memcg = mem_cgroup_iter(NULL, memcg, NULL)));
unlock:
	mutex_unlock(&state_mutex);
	put_online_mems();
	cpus_read_unlock();
	cgroup_unlock();
}

/******************************************************************************
 *                          sysfs interface
 ******************************************************************************/

static ssize_t min_ttl_ms_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%u\n", jiffies_to_msecs(READ_ONCE(lru_gen_min_ttl)));
}

/* see Documentation/admin-guide/mm/multigen_lru.rst for details */
/* 这个文档没有相关解释 */
static ssize_t min_ttl_ms_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t len)
{
	unsigned int msecs;

	if (kstrtouint(buf, 0, &msecs))
		return -EINVAL;

	WRITE_ONCE(lru_gen_min_ttl, msecs_to_jiffies(msecs));

	return len;
}

static struct kobj_attribute lru_gen_min_ttl_attr = __ATTR_RW(min_ttl_ms);

static ssize_t enabled_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	unsigned int caps = 0;

	if (get_cap(LRU_GEN_CORE))
		caps |= BIT(LRU_GEN_CORE);

	if (should_walk_mmu())
		caps |= BIT(LRU_GEN_MM_WALK);

	if (should_clear_pmd_young())
		caps |= BIT(LRU_GEN_NONLEAF_YOUNG);

	return sysfs_emit(buf, "0x%04x\n", caps);
}

/* see Documentation/admin-guide/mm/multigen_lru.rst for details */
static ssize_t enabled_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t len)
{
	int i;
	unsigned int caps;

	if (tolower(*buf) == 'n')
		caps = 0;
	else if (tolower(*buf) == 'y')
		caps = -1;
	else if (kstrtouint(buf, 0, &caps))
		return -EINVAL;

	for (i = 0; i < NR_LRU_GEN_CAPS; i++) {
		bool enabled = caps & BIT(i);

		if (i == LRU_GEN_CORE)
			lru_gen_change_state(enabled);
		else if (enabled)
			static_branch_enable(&lru_gen_caps[i]);
		else
			static_branch_disable(&lru_gen_caps[i]);
	}

	return len;
}

static struct kobj_attribute lru_gen_enabled_attr = __ATTR_RW(enabled);

static struct attribute *lru_gen_attrs[] = {
	&lru_gen_min_ttl_attr.attr,
	&lru_gen_enabled_attr.attr,
	NULL
};

static const struct attribute_group lru_gen_attr_group = {
	.name = "lru_gen",
	.attrs = lru_gen_attrs,
};

/******************************************************************************
 *                          debugfs interface
 ******************************************************************************/

static void *lru_gen_seq_start(struct seq_file *m, loff_t *pos)
{
	struct mem_cgroup *memcg;
	loff_t nr_to_skip = *pos;

	m->private = kvmalloc(PATH_MAX, GFP_KERNEL);
	if (!m->private)
		return ERR_PTR(-ENOMEM);

	memcg = mem_cgroup_iter(NULL, NULL, NULL);
	do {
		int nid;

		for_each_node_state(nid, N_MEMORY) {
			if (!nr_to_skip--)
				return get_lruvec(memcg, nid);
		}
	} while ((memcg = mem_cgroup_iter(NULL, memcg, NULL)));

	return NULL;
}

static void lru_gen_seq_stop(struct seq_file *m, void *v)
{
	if (!IS_ERR_OR_NULL(v))
		mem_cgroup_iter_break(NULL, lruvec_memcg(v));

	kvfree(m->private);
	m->private = NULL;
}

static void *lru_gen_seq_next(struct seq_file *m, void *v, loff_t *pos)
{
	int nid = lruvec_pgdat(v)->node_id;
	struct mem_cgroup *memcg = lruvec_memcg(v);

	++*pos;

	nid = next_memory_node(nid);
	if (nid == MAX_NUMNODES) {
		memcg = mem_cgroup_iter(NULL, memcg, NULL);
		if (!memcg)
			return NULL;

		nid = first_memory_node;
	}

	return get_lruvec(memcg, nid);
}

static void lru_gen_seq_show_full(struct seq_file *m, struct lruvec *lruvec,
				  unsigned long max_seq, unsigned long *min_seq,
				  unsigned long seq)
{
	int i;
	int type, tier;
	int hist = lru_hist_from_seq(seq);
	struct lru_gen_folio *lrugen = &lruvec->lrugen;
	struct lru_gen_mm_state *mm_state = get_mm_state(lruvec);

	for (tier = 0; tier < MAX_NR_TIERS; tier++) {
		seq_printf(m, "            %10d", tier);
		for (type = 0; type < ANON_AND_FILE; type++) {
			const char *s = "xxx";
			unsigned long n[3] = {};

			if (seq == max_seq) {
				s = "RTx";
				n[0] = READ_ONCE(lrugen->avg_refaulted[type][tier]);
				n[1] = READ_ONCE(lrugen->avg_total[type][tier]);
			} else if (seq == min_seq[type] || NR_HIST_GENS > 1) {
				s = "rep";
				n[0] = atomic_long_read(&lrugen->refaulted[hist][type][tier]);
				n[1] = atomic_long_read(&lrugen->evicted[hist][type][tier]);
				if (tier)
					n[2] = READ_ONCE(lrugen->protected[hist][type][tier - 1]);
			}

			for (i = 0; i < 3; i++)
				seq_printf(m, " %10lu%c", n[i], s[i]);
		}
		seq_putc(m, '\n');
	}

	if (!mm_state)
		return;

	seq_puts(m, "                      ");
	for (i = 0; i < NR_MM_STATS; i++) {
		const char *s = "xxxx";
		unsigned long n = 0;

		if (seq == max_seq && NR_HIST_GENS == 1) {
			s = "TYFA";
			n = READ_ONCE(mm_state->stats[hist][i]);
		} else if (seq != max_seq && NR_HIST_GENS > 1) {
			s = "tyfa";
			n = READ_ONCE(mm_state->stats[hist][i]);
		}

		seq_printf(m, " %10lu%c", n, s[i]);
	}
	seq_putc(m, '\n');
}

/* see Documentation/admin-guide/mm/multigen_lru.rst for details */
static int lru_gen_seq_show(struct seq_file *m, void *v)
{
	unsigned long seq;
	bool full = !debugfs_real_fops(m->file)->write;
	struct lruvec *lruvec = v;
	struct lru_gen_folio *lrugen = &lruvec->lrugen;
	int nid = lruvec_pgdat(lruvec)->node_id;
	struct mem_cgroup *memcg = lruvec_memcg(lruvec);
	DEFINE_MAX_SEQ(lruvec);
	DEFINE_MIN_SEQ(lruvec);

	if (nid == first_memory_node) {
		const char *path = memcg ? m->private : "";

#ifdef CONFIG_MEMCG
		if (memcg)
			cgroup_path(memcg->css.cgroup, m->private, PATH_MAX);
#endif
		seq_printf(m, "memcg %5hu %s\n", mem_cgroup_id(memcg), path);
	}

	seq_printf(m, " node %5d\n", nid);

	if (!full)
		seq = min_seq[LRU_GEN_ANON];
	else if (max_seq >= MAX_NR_GENS)
		seq = max_seq - MAX_NR_GENS + 1;
	else
		seq = 0;

	for (; seq <= max_seq; seq++) {
		int type, zone;
		int gen = lru_gen_from_seq(seq);
		unsigned long birth = READ_ONCE(lruvec->lrugen.timestamps[gen]);

		seq_printf(m, " %10lu %10u", seq, jiffies_to_msecs(jiffies - birth));

		for (type = 0; type < ANON_AND_FILE; type++) {
			unsigned long size = 0;
			char mark = full && seq < min_seq[type] ? 'x' : ' ';

			for (zone = 0; zone < MAX_NR_ZONES; zone++)
				size += max(READ_ONCE(lrugen->nr_pages[gen][type][zone]), 0L);

			seq_printf(m, " %10lu%c", size, mark);
		}

		seq_putc(m, '\n');

		if (full)
			lru_gen_seq_show_full(m, lruvec, max_seq, min_seq, seq);
	}

	return 0;
}

static const struct seq_operations lru_gen_seq_ops = {
	.start = lru_gen_seq_start,
	.stop = lru_gen_seq_stop,
	.next = lru_gen_seq_next,
	.show = lru_gen_seq_show,
};

static int run_aging(struct lruvec *lruvec, unsigned long seq,
		     bool can_swap, bool force_scan)
{
	DEFINE_MAX_SEQ(lruvec);
	DEFINE_MIN_SEQ(lruvec);

	if (seq < max_seq)
		return 0;

	if (seq > max_seq)
		return -EINVAL;

	if (!force_scan && min_seq[!can_swap] + MAX_NR_GENS - 1 <= max_seq)
		return -ERANGE;

	try_to_inc_max_seq(lruvec, max_seq, can_swap, force_scan);

	return 0;
}

static int run_eviction(struct lruvec *lruvec, unsigned long seq, struct scan_control *sc,
			int swappiness, unsigned long nr_to_reclaim)
{
	DEFINE_MAX_SEQ(lruvec);

	if (seq + MIN_NR_GENS > max_seq)
		return -EINVAL;

	sc->nr_reclaimed = 0;

	while (!signal_pending(current)) {
		DEFINE_MIN_SEQ(lruvec);

		if (seq < min_seq[!swappiness])
			return 0;

		if (sc->nr_reclaimed >= nr_to_reclaim)
			return 0;

		if (!evict_folios(lruvec, sc, swappiness))
			return 0;

		cond_resched();
	}

	return -EINTR;
}

static int run_cmd(char cmd, int memcg_id, int nid, unsigned long seq,
		   struct scan_control *sc, int swappiness, unsigned long opt)
{
	struct lruvec *lruvec;
	int err = -EINVAL;
	struct mem_cgroup *memcg = NULL;

	if (nid < 0 || nid >= MAX_NUMNODES || !node_state(nid, N_MEMORY))
		return -EINVAL;

	if (!mem_cgroup_disabled()) {
		rcu_read_lock();

		memcg = mem_cgroup_from_id(memcg_id);
		if (!mem_cgroup_tryget(memcg))
			memcg = NULL;

		rcu_read_unlock();

		if (!memcg)
			return -EINVAL;
	}

	if (memcg_id != mem_cgroup_id(memcg))
		goto done;

	lruvec = get_lruvec(memcg, nid);

	if (swappiness < MIN_SWAPPINESS)
		swappiness = get_swappiness(lruvec, sc);
	else if (swappiness > MAX_SWAPPINESS)
		goto done;

	switch (cmd) {
	case '+':
		err = run_aging(lruvec, seq, swappiness, opt);
		break;
	case '-':
		err = run_eviction(lruvec, seq, sc, swappiness, opt);
		break;
	}
done:
	mem_cgroup_put(memcg);

	return err;
}

/* see Documentation/admin-guide/mm/multigen_lru.rst for details */
static ssize_t lru_gen_seq_write(struct file *file, const char __user *src,
				 size_t len, loff_t *pos)
{
	void *buf;
	char *cur, *next;
	unsigned int flags;
	struct blk_plug plug;
	int err = -EINVAL;
	struct scan_control sc = {
		.may_writepage = true,
		.may_unmap = true,
		.may_swap = true,
		.reclaim_idx = MAX_NR_ZONES - 1,
		.gfp_mask = GFP_KERNEL,
	};

	buf = kvmalloc(len + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, src, len)) {
		kvfree(buf);
		return -EFAULT;
	}

	set_task_reclaim_state(current, &sc.reclaim_state);
	flags = memalloc_noreclaim_save();
	blk_start_plug(&plug);
	if (!set_mm_walk(NULL, true)) {
		err = -ENOMEM;
		goto done;
	}

	next = buf;
	next[len] = '\0';

	while ((cur = strsep(&next, ",;\n"))) {
		int n;
		int end;
		char cmd;
		unsigned int memcg_id;
		unsigned int nid;
		unsigned long seq;
		unsigned int swappiness = -1;
		unsigned long opt = -1;

		cur = skip_spaces(cur);
		if (!*cur)
			continue;

		n = sscanf(cur, "%c %u %u %lu %n %u %n %lu %n", &cmd, &memcg_id, &nid,
			   &seq, &end, &swappiness, &end, &opt, &end);
		if (n < 4 || cur[end]) {
			err = -EINVAL;
			break;
		}

		err = run_cmd(cmd, memcg_id, nid, seq, &sc, swappiness, opt);
		if (err)
			break;
	}
done:
	clear_mm_walk();
	blk_finish_plug(&plug);
	memalloc_noreclaim_restore(flags);
	set_task_reclaim_state(current, NULL);

	kvfree(buf);

	return err ? : len;
}

static int lru_gen_seq_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &lru_gen_seq_ops);
}

static const struct file_operations lru_gen_rw_fops = {
	.open = lru_gen_seq_open,
	.read = seq_read,
	.write = lru_gen_seq_write,
	.llseek = seq_lseek,
	.release = seq_release,
};

static const struct file_operations lru_gen_ro_fops = {
	.open = lru_gen_seq_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

/******************************************************************************
 *                          initialization
 ******************************************************************************/

void lru_gen_init_pgdat(struct pglist_data *pgdat)
{
	int i, j;

	spin_lock_init(&pgdat->memcg_lru.lock);

	for (i = 0; i < MEMCG_NR_GENS; i++) {
		for (j = 0; j < MEMCG_NR_BINS; j++)
			INIT_HLIST_NULLS_HEAD(&pgdat->memcg_lru.fifo[i][j], i);
	}
}

/*
 * MGLRU初始化lruvec
 */
void lru_gen_init_lruvec(struct lruvec *lruvec)
{
	int i;
	int gen, type, zone;
	struct lru_gen_folio *lrugen = &lruvec->lrugen;
	struct lru_gen_mm_state *mm_state = get_mm_state(lruvec);

	/* max_seq = 3 */
	lrugen->max_seq = MIN_NR_GENS + 1;
	lrugen->enabled = lru_gen_enabled();

	for (i = 0; i <= MIN_NR_GENS + 1; i++)
		lrugen->timestamps[i] = jiffies;

	for_each_gen_type_zone(gen, type, zone)
		INIT_LIST_HEAD(&lrugen->folios[gen][type][zone]);

	if (mm_state)
		mm_state->seq = MIN_NR_GENS;
}

#ifdef CONFIG_MEMCG

void lru_gen_init_memcg(struct mem_cgroup *memcg)
{
	struct lru_gen_mm_list *mm_list = get_mm_list(memcg);

	if (!mm_list)
		return;

	INIT_LIST_HEAD(&mm_list->fifo);
	spin_lock_init(&mm_list->lock);
}

void lru_gen_exit_memcg(struct mem_cgroup *memcg)
{
	int i;
	int nid;
	struct lru_gen_mm_list *mm_list = get_mm_list(memcg);

	VM_WARN_ON_ONCE(mm_list && !list_empty(&mm_list->fifo));

	for_each_node(nid) {
		struct lruvec *lruvec = get_lruvec(memcg, nid);
		struct lru_gen_mm_state *mm_state = get_mm_state(lruvec);

		VM_WARN_ON_ONCE(memchr_inv(lruvec->lrugen.nr_pages, 0,
					   sizeof(lruvec->lrugen.nr_pages)));

		lruvec->lrugen.list.next = LIST_POISON1;

		if (!mm_state)
			continue;

		for (i = 0; i < NR_BLOOM_FILTERS; i++) {
			bitmap_free(mm_state->filters[i]);
			mm_state->filters[i] = NULL;
		}
	}
}

#endif /* CONFIG_MEMCG */

static int __init init_lru_gen(void)
{
	BUILD_BUG_ON(MIN_NR_GENS + 1 >= MAX_NR_GENS);
	BUILD_BUG_ON(BIT(LRU_GEN_WIDTH) <= MAX_NR_GENS);

	if (sysfs_create_group(mm_kobj, &lru_gen_attr_group))
		pr_err("lru_gen: failed to create sysfs group\n");

	debugfs_create_file("lru_gen", 0644, NULL, NULL, &lru_gen_rw_fops);
	debugfs_create_file("lru_gen_full", 0444, NULL, NULL, &lru_gen_ro_fops);

	return 0;
};
late_initcall(init_lru_gen);

#else /* !CONFIG_LRU_GEN */

static void lru_gen_age_node(struct pglist_data *pgdat, struct scan_control *sc)
{
	BUILD_BUG();
}

static void lru_gen_shrink_lruvec(struct lruvec *lruvec, struct scan_control *sc)
{
	BUILD_BUG();
}

static void lru_gen_shrink_node(struct pglist_data *pgdat, struct scan_control *sc)
{
	BUILD_BUG();
}

#endif /* CONFIG_LRU_GEN */

/*
 * 计算需要扫描的页面数量，并遍历各类LRU可回收链表，依次回收/老化页面
 *
 * 1.使能MGLRU且为memcg局部回收，则走lru_gen_shrink_lruvec路径；
 * 2.调用**get_scan_count**，确认每个可回收LRU链表的页面扫描数量；
 * 3.针对全局直接回收、且还是默认回收优先级，设置proportional_reclaim;
 * 4.遍历可回收的LRU链表，依次调用**shrink_list**老化/回收页面;
 *	- 设置每次扫描的最大数量为32（SWAP_CLUSTER_MAX）;
 *	- 如果设置了proportional_reclaim, 一直回收，直到页面全部扫描完后，直接退出；
 *	- 如果kswapd或者memcg局部回收等, 达成回收木匾后，根据剩余扫描量重新调整扫描量，直到数量扫描完成后退出；
 * 5.如果inactive页面数量较少，调用**shrink_active_list**，再次平衡匿名链表的active/inactive比例；
 */
static void shrink_lruvec(struct lruvec *lruvec, struct scan_control *sc)
{
	unsigned long nr[NR_LRU_LISTS];		/* 存放不同LRU类型需要扫描的页面数量 */
	unsigned long targets[NR_LRU_LISTS];
	unsigned long nr_to_scan;
	enum lru_list lru;
	unsigned long nr_reclaimed = 0;
	unsigned long nr_to_reclaim = sc->nr_to_reclaim;
	bool proportional_reclaim;
	struct blk_plug plug;

        /*
         * 使能MGLRU且非全局回收（通常为memcg局部回收）时，交由LRU_GEN路径处理：
         * - 基于访问代际的信息区分冷热，能更精准命中冷页，减少误回收。
	 */
	if (lru_gen_enabled() && !root_reclaim(sc)) {
		lru_gen_shrink_lruvec(lruvec, sc);
		return;
	}

	/* 关键：确定每个可回收列表的扫描页面的数量，保存到nr数组中 */
	get_scan_count(lruvec, sc, nr);

	/* Record the original scan target for proportional adjustments later */
	memcpy(targets, nr, sizeof(nr));

	/*
	 * Global reclaiming within direct reclaim at DEF_PRIORITY is a normal
	 * event that can occur when there is little memory pressure e.g.
	 * multiple streaming readers/writers. Hence, we do not abort scanning
	 * when the requested number of pages are reclaimed when scanning at
	 * DEF_PRIORITY on the assumption that the fact we are direct
	 * reclaiming implies that kswapd is not keeping up and it is best to
	 * do a batch of work at once. For memcg reclaim one check is made to
	 * abort proportional reclaim if either the file or anon lru has already
	 * dropped to zero at the first pass.
	 */
	/*
	 * 针对全局直接回收、且还是默认回收优先级的特殊处理
	 *
         * 在直接回收中，DEF_PRIORITY级别的全局回收是一个正常事件，
	 * 在这种情况下，认为kswapd可能更不太上，所以可以多回收一些页面，
	 * 即使回收到目标页面数，也继续回收，直到页面全部扫描完（下面的while判断条件）
	 * 所以设置proportional_reclaim，确保这种情况下，能继续回收。
	 */
	proportional_reclaim = (!cgroup_reclaim(sc) && !current_is_kswapd() &&
				sc->priority == DEF_PRIORITY);

        /*
	 * 开始块设备插桩，优化后续的页面换出I/O
         * 合并块I/O，降低小批次写回/回收造成的I/O抖动
	 */
	blk_start_plug(&plug);
	while (nr[LRU_INACTIVE_ANON] || nr[LRU_ACTIVE_FILE] ||
					nr[LRU_INACTIVE_FILE]) {
		unsigned long nr_anon, nr_file, percentage;
		unsigned long nr_scanned;

		/*
		 * 遍历可回收的LRU链表，调用shrink_list依次回收内存，
		 * 每次扫描页面的最大数量为SWAP_CLUSTER_MAX(32)
		 * active链表主要是为老化/降级，inactive链表为回收。
		 */
		for_each_evictable_lru(lru) {
			if (nr[lru]) {
				nr_to_scan = min(nr[lru], SWAP_CLUSTER_MAX);
				nr[lru] -= nr_to_scan;

				/* 回收/老化LRU链表，并记录实际回收的页面数量 */
				nr_reclaimed += shrink_list(lru, nr_to_scan,
							    lruvec, sc);
			}
		}

		cond_resched();

		/*
		 * 1.全局的DEF_PRIORITY直接回收，到这里返回继续，直到全部页面扫描完毕后, 直接退出;
		 * 2.kswapd或者memcg等，还没回收目标数量，且页面没扫描完，则返回继续回收，到达后往下继续;
		 */
		if (nr_reclaimed < nr_to_reclaim || proportional_reclaim)
			continue;

		/* 达到回收目标后，根据剩余扫描量，按比例，重新调整扫描页面数量(减少) */
		/*
		 * For kswapd and memcg, reclaim at least the number of pages
		 * requested. Ensure that the anon and file LRUs are scanned
		 * proportionally what was requested by get_scan_count(). We
		 * stop reclaiming one LRU and reduce the amount scanning
		 * proportional to the original scan target.
		 */
		nr_file = nr[LRU_INACTIVE_FILE] + nr[LRU_ACTIVE_FILE];
		nr_anon = nr[LRU_INACTIVE_ANON] + nr[LRU_ACTIVE_ANON];

		/*
		 * It's just vindictive to attack the larger once the smaller
		 * has gone to zero.  And given the way we stop scanning the
		 * smaller below, this makes sure that we only make one nudge
		 * towards proportionality once we've got nr_to_reclaim.
		 */
		if (!nr_file || !nr_anon)
			break;

                /* 根据剩余扫描数量决定调整哪个LRU的比例 */
		if (nr_file > nr_anon) {
                        /* 剩余文件页更多，调整匿名页的扫描 */
			unsigned long scan_target = targets[LRU_INACTIVE_ANON] +
						targets[LRU_ACTIVE_ANON] + 1;
			lru = LRU_BASE;
			percentage = nr_anon * 100 / scan_target;
		} else {
                        /* 匿名页更多，调整文件页的扫描 */
			unsigned long scan_target = targets[LRU_INACTIVE_FILE] +
						targets[LRU_ACTIVE_FILE] + 1;
			lru = LRU_FILE;
			percentage = nr_file * 100 / scan_target;
		}

		/* Stop scanning the smaller of the LRU */
                /* 停止扫描较小的这类（该类active与inactive均置0） */
		nr[lru] = 0;
		nr[lru + LRU_ACTIVE] = 0;

		/*
		 * Recalculate the other LRU scan count based on its original
		 * scan target and the percentage scanning already complete
		 */
                /*
                 * 对另一类按“已完成百分比”缩减剩余扫描量：
                 * 基于其原始扫描目标和已完成的扫描百分比重新计算另一个 LRU 的扫描计数
                 * - 以targets为基线，按百分比减少，且扣掉已扫描量，避免负数。
                 */
		lru = (lru == LRU_FILE) ? LRU_BASE : LRU_FILE;
		nr_scanned = targets[lru] - nr[lru];			/* 已经扫描的数量 */
		nr[lru] = targets[lru] * (100 - percentage) / 100;	/* 调整剩余扫描量 */
		nr[lru] -= min(nr[lru], nr_scanned);			/* 减去已扫描的部分 */

                /* 同样调整活跃链表 */
		lru += LRU_ACTIVE;
		nr_scanned = targets[lru] - nr[lru];
		nr[lru] = targets[lru] * (100 - percentage) / 100;
		nr[lru] -= min(nr[lru], nr_scanned);
	}
	blk_finish_plug(&plug);
	sc->nr_reclaimed += nr_reclaimed;

	/*
	 * Even if we did not try to evict anon pages at all, we want to
	 * rebalance the anon lru active/inactive ratio.
	 */
	/*
         * 平衡匿名LRU链表的活跃/非活跃比率
         *   即便本轮没有尝试回收匿名页，也希望维护anon active/inactive的合理比例，
         *   当inactive_anon偏低且“匿名可老化”（有swap或可NUMA降级）时，推一小批活跃匿名页
         *   向不活跃，便于后续真正回收（回收从inactive开始）。
	 */
	if (can_age_anon_pages(lruvec_pgdat(lruvec), sc) &&
	    inactive_is_low(lruvec, LRU_INACTIVE_ANON))
		shrink_active_list(SWAP_CLUSTER_MAX, lruvec,
				   sc, LRU_ACTIVE_ANON);
}

/* Use reclaim/compaction for costly allocs or under memory pressure */
static bool in_reclaim_compaction(struct scan_control *sc)
{
	if (gfp_compaction_allowed(sc->gfp_mask) && sc->order &&
			(sc->order > PAGE_ALLOC_COSTLY_ORDER ||
			 sc->priority < DEF_PRIORITY - 2))
		return true;

	return false;
}

/*
 * Reclaim/compaction is used for high-order allocation requests. It reclaims
 * order-0 pages before compacting the zone. should_continue_reclaim() returns
 * true if more pages should be reclaimed such that when the page allocator
 * calls try_to_compact_pages() that it will have enough free pages to succeed.
 * It will give up earlier than that if there is difficulty reclaiming pages.
 */
static inline bool should_continue_reclaim(struct pglist_data *pgdat,
					unsigned long nr_reclaimed,
					struct scan_control *sc)
{
	unsigned long pages_for_compaction;
	unsigned long inactive_lru_pages;
	int z;

	/* If not in reclaim/compaction mode, stop */
	if (!in_reclaim_compaction(sc))
		return false;

	/*
	 * Stop if we failed to reclaim any pages from the last SWAP_CLUSTER_MAX
	 * number of pages that were scanned. This will return to the caller
	 * with the risk reclaim/compaction and the resulting allocation attempt
	 * fails. In the past we have tried harder for __GFP_RETRY_MAYFAIL
	 * allocations through requiring that the full LRU list has been scanned
	 * first, by assuming that zero delta of sc->nr_scanned means full LRU
	 * scan, but that approximation was wrong, and there were corner cases
	 * where always a non-zero amount of pages were scanned.
	 */
	if (!nr_reclaimed)
		return false;

	/* If compaction would go ahead or the allocation would succeed, stop */
	for (z = 0; z <= sc->reclaim_idx; z++) {
		struct zone *zone = &pgdat->node_zones[z];
		if (!managed_zone(zone))
			continue;

		/* Allocation can already succeed, nothing to do */
		if (zone_watermark_ok(zone, sc->order, min_wmark_pages(zone),
				      sc->reclaim_idx, 0))
			return false;

		if (compaction_suitable(zone, sc->order, sc->reclaim_idx))
			return false;
	}

	/*
	 * If we have not reclaimed enough pages for compaction and the
	 * inactive lists are large enough, continue reclaiming
	 */
	pages_for_compaction = compact_gap(sc->order);
	inactive_lru_pages = node_page_state(pgdat, NR_INACTIVE_FILE);
	if (can_reclaim_anon_pages(NULL, pgdat->node_id, sc))
		inactive_lru_pages += node_page_state(pgdat, NR_INACTIVE_ANON);

	return inactive_lru_pages > pages_for_compaction;
}

/*
 * 遍历memcg，对每个memcg进行LRU回收和slab回收
 *
 * 1.遍历memcg，如果是kswapd则进行完整遍历, 反之则进行部分遍历;
 * 2.判断当前memcg的内存保护情况，判断是否回收；
 *	- 如果当前memcg内存使用低于min限制，则需要硬保护, 禁止回收，跳过；
 *	- 如果当前memcg内存使用低于low限制，则需要软保护，条件回收；
 *		+ 只有设置了sc->memcg_low_reclaim，才能继续回收；
 * 3.调用**shrink_lruve**，遍历所有可回收的LRU链表，进行老化/回收；
 * 4.调用**shrink_slab**，回收slab内存；
 * 5.如果是完整编译，则需要完整操作完所有memcg才退出，如果是部分遍历，只要达成回收目标即可退出；
 */
static void shrink_node_memcgs(pg_data_t *pgdat, struct scan_control *sc)
{
	struct mem_cgroup *target_memcg = sc->target_mem_cgroup;
	struct mem_cgroup_reclaim_cookie reclaim = {
		.pgdat = pgdat,
	};
	struct mem_cgroup_reclaim_cookie *partial = &reclaim;
	struct mem_cgroup *memcg;

	/*
	 * In most cases, direct reclaimers can do partial walks
	 * through the cgroup tree, using an iterator state that
	 * persists across invocations. This strikes a balance between
	 * fairness and allocation latency.
	 *
	 * For kswapd, reliable forward progress is more important
	 * than a quick return to idle. Always do full walks.
	 */
	/* 直接回收进行memcg部分遍历, kswapd总是进行完整遍历 */
	if (current_is_kswapd() || sc->memcg_full_walk)
		partial = NULL;

	memcg = mem_cgroup_iter(target_memcg, NULL, partial);
	do {
		struct lruvec *lruvec = mem_cgroup_lruvec(memcg, pgdat);
		unsigned long reclaimed;
		unsigned long scanned;

		/*
		 * This loop can become CPU-bound when target memcgs
		 * aren't eligible for reclaim - either because they
		 * don't have any reclaimable pages, or because their
		 * memory is explicitly protected. Avoid soft lockups.
		 */
		cond_resched();

		mem_cgroup_calculate_protection(target_memcg, memcg);

		/* 当前memcg内存使用低于min限制，则需要硬保护, 禁止回收 */
		if (mem_cgroup_below_min(target_memcg, memcg)) {
			/*
			 * Hard protection.
			 * If there is no reclaimable memory, OOM.
			 */
			continue;
		/* 当前memcg内存使用低于low限制，则需要软保护, 条件回收 */
		} else if (mem_cgroup_below_low(target_memcg, memcg)) {
			/*
			 * Soft protection.
			 * Respect the protection only as long as
			 * there is an unprotected supply
			 * of reclaimable memory from other cgroups.
			 */
			if (!sc->memcg_low_reclaim) {
				sc->memcg_low_skipped = 1;
				continue;
			}
			memcg_memory_event(memcg, MEMCG_LOW);
		}

		reclaimed = sc->nr_reclaimed;
		scanned = sc->nr_scanned;

		/* 关键：遍历LRU各类可回收链表，依次回收/老化页面 */
		shrink_lruvec(lruvec, sc);

		/* 调用注册的shrinker回收slab内存 */
		shrink_slab(sc->gfp_mask, pgdat->node_id, memcg,
			    sc->priority);

		/* Record the group's reclaim efficiency */
		if (!sc->proactive)
			vmpressure(sc->gfp_mask, memcg, false,
				   sc->nr_scanned - scanned,
				   sc->nr_reclaimed - reclaimed);

		/* If partial walks are allowed, bail once goal is reached */
                /* 如果允许部分遍历，一旦达到目标就退出 */
		if (partial && sc->nr_reclaimed >= sc->nr_to_reclaim) {
			mem_cgroup_iter_break(target_memcg, memcg);
			break;
		}
	} while ((memcg = mem_cgroup_iter(target_memcg, memcg, partial)));
}

/*
 * 回收内存节点内存
 *
 * 1.如果使能了MGLRU，并且是全局回收，则走MGLRU的shrink流程;
 * 2.更新sc参数；
 * 3.遍历memcg，对每个memcg进行LRU回收和slab回收; **关键**
 * 4.更新全局回收状态;
 * 5.如果是kswap流程，且隔离的页面都是writeback页面，则节流休眠；
 * 6.如果是直接回收并且处于拥塞状态（congested），则节流休眠；
 */
static void shrink_node(pg_data_t *pgdat, struct scan_control *sc)
{
	unsigned long nr_reclaimed, nr_scanned, nr_node_reclaimed;
	struct lruvec *target_lruvec;
	bool reclaimable = false;

	/* 如果使能了MGLRU，并且是全局回收，则走MGLRU的shrink流程 */
	if (lru_gen_enabled() && root_reclaim(sc)) {
		lru_gen_shrink_node(pgdat, sc);
		return;
	}

	target_lruvec = mem_cgroup_lruvec(sc->target_mem_cgroup, pgdat);

again:
	memset(&sc->nr, 0, sizeof(sc->nr));

	nr_reclaimed = sc->nr_reclaimed;
	nr_scanned = sc->nr_scanned;

	/* 更新sc参数，包括:
	 *	- anon_cost\file_cost
	 *	- may_deactivate
	 *	- cache_trim_mode
	 *	- file_is_tiny
	 */
	prepare_scan_control(pgdat, sc);

	/* 遍历memcg，对每个memcg进行LRU回收和slab回收 */
	shrink_node_memcgs(pgdat, sc);

	/* 更新全局回收状态 */
	flush_reclaim_state(sc);

	/* 计算本次回收的页面数量 */
	nr_node_reclaimed = sc->nr_reclaimed - nr_reclaimed;

	/* Record the subtree's reclaim efficiency */
	if (!sc->proactive)
		vmpressure(sc->gfp_mask, sc->target_mem_cgroup, true,
			   sc->nr_scanned - nr_scanned, nr_node_reclaimed);

	if (nr_node_reclaimed)
		reclaimable = true;

	if (current_is_kswapd()) {
		/*
		 * If reclaim is isolating dirty pages under writeback,
		 * it implies that the long-lived page allocation rate
		 * is exceeding the page laundering rate. Either the
		 * global limits are not being effective at throttling
		 * processes due to the page distribution throughout
		 * zones or there is heavy usage of a slow backing
		 * device. The only option is to throttle from reclaim
		 * context which is not ideal as there is no guarantee
		 * the dirtying process is throttled in the same way
		 * balance_dirty_pages() manages.
		 *
		 * Once a node is flagged PGDAT_WRITEBACK, kswapd will
		 * count the number of pages under pages flagged for
		 * immediate reclaim and stall if any are encountered
		 * in the nr_immediate check below.
		 */
		/*
		 * 如果kswapd回收时，隔离出来的页面都是writeback页面，
		 * 说明长生命周期页面的分类速度超过页面清理速度
                 * 说明可能是全局限制因页面在 zones 间的分布而无法有效
                 * 限制进程，或者是慢速存储设备被重度使用。唯一的选项是在回收上下文中限制，
                 * 这并不理想，因为无法保证脏页进程会以 balance_dirty_pages() 管理的
                 * 相同方式被限制。
                 *
                 * 一旦节点被标记为 PGDAT_WRITEBACK，kswapd 将统计被标记为立即回收的
                 * 页面数量，并在下面的 nr_immediate 检查中遇到任何此类页面时停止。
		 *
		 * 回收流程中如果遇到正在writeback的页面，则会统计到sc->nr.writeback(shrink_folio_list)
		 * sc->nr.taken是隔离出来的实际页面数量(isolate_lru_folios)
		 *
		 * 在kswapd回收流程中(shrink_folio_list)，如果检测到页面是writeback且设置了PGDAT_WRITEBACK，
		 * 则将这些页面激活，避免在LRU inactive链表上堆积，并会将其统计到sc->nr.immediate中
		 */
		if (sc->nr.writeback && sc->nr.writeback == sc->nr.taken)
			set_bit(PGDAT_WRITEBACK, &pgdat->flags);

		/* Allow kswapd to start writing pages during reclaim.*/
		/*
		 * sc->nr.unqueued_dirty: 回收流程中，统计ditry但未加入回写队列的页面数量
		 * sc->nr.file_taken: 回收流程中，隔离出来的文件页面数量 (shrink_inactive_list)
		 *
		 * 如果隔离出来的文件页面都是dirty但未加入回写队列的，则允许kswapd在回收期间开始回写页面
		 */
		if (sc->nr.unqueued_dirty == sc->nr.file_taken)
			set_bit(PGDAT_DIRTY, &pgdat->flags);

		/*
		 * If kswapd scans pages marked for immediate
		 * reclaim and under writeback (nr_immediate), it
		 * implies that pages are cycling through the LRU
		 * faster than they are written so forcibly stall
		 * until some pages complete writeback.
		 */
		/*
		 * sc->nr.immediate页面表示正在writeback的页面，
                 * 意味着页面在LRU中循环的速度快于它们被写回的速度，
		 * 因此节流休眠直到回写完成
		 *
		 * 在too_many_isolated()中判断隔离页面不会过多，则唤醒
		 */
		if (sc->nr.immediate)
			reclaim_throttle(pgdat, VMSCAN_THROTTLE_WRITEBACK);
	}

	/*
	 * Tag a node/memcg as congested if all the dirty pages were marked
	 * for writeback and immediate reclaim (counted in nr.congested).
	 *
	 * Legacy memcg will stall in page writeback so avoid forcibly
	 * stalling in reclaim_throttle().
	 */
        /*
         * 拥塞检测：如果所有脏页都被标记为写回和立即回收，
         * 则标记node/memcg为拥塞状态，用于后续的节流决策
         */
	if (sc->nr.dirty && sc->nr.dirty == sc->nr.congested) {
		if (cgroup_reclaim(sc) && writeback_throttling_sane(sc))
			set_bit(LRUVEC_CGROUP_CONGESTED, &target_lruvec->flags);

		if (current_is_kswapd())
			set_bit(LRUVEC_NODE_CONGESTED, &target_lruvec->flags);
	}

	/*
	 * Stall direct reclaim for IO completions if the lruvec is (注释有误，去掉is)
	 * node is congested. Allow kswapd to continue until it
	 * starts encountering unqueued dirty pages or cycling through
	 * the LRU too quickly.
	 */
        /*
         * 直接回收的节流：如果lruvec处于拥塞状态，节流直接回收
         * 等待IO完成，但允许kswapd继续直到遇到未排队的脏页或LRU循环过快
         */
	if (!current_is_kswapd() && current_may_throttle() &&
	    !sc->hibernation_mode &&
	    (test_bit(LRUVEC_CGROUP_CONGESTED, &target_lruvec->flags) ||
	     test_bit(LRUVEC_NODE_CONGESTED, &target_lruvec->flags)))
		reclaim_throttle(pgdat, VMSCAN_THROTTLE_CONGESTED);

	/* 判断是否能继续回收，细节待研究 */
	if (should_continue_reclaim(pgdat, nr_node_reclaimed, sc))
		goto again;

	/*
	 * Kswapd gives up on balancing particular nodes after too
	 * many failures to reclaim anything from them and goes to
	 * sleep. On reclaim progress, reset the failure counter. A
	 * successful direct reclaim run will revive a dormant kswapd.
	 */
	/*
	 * 成功回收到页面，则将kswapd失败计数置0
         * 如果cache_trim_mode回收失败，标记失败标志
	 */
	if (reclaimable)
		pgdat->kswapd_failures = 0;
	else if (sc->cache_trim_mode)
		sc->cache_trim_mode_failed = 1;
}

/*
 * Returns true if compaction should go ahead for a costly-order request, or
 * the allocation would already succeed without compaction. Return false if we
 * should reclaim first.
 */
/*
 * 根据当前zone的空闲内存，判断能否进行内存规整
 */
static inline bool compaction_ready(struct zone *zone, struct scan_control *sc)
{
	unsigned long watermark;

	if (!gfp_compaction_allowed(sc->gfp_mask))
		return false;

	/* Allocation can already succeed, nothing to do */
	if (zone_watermark_ok(zone, sc->order, min_wmark_pages(zone),
			      sc->reclaim_idx, 0))
		return true;

	/* Compaction cannot yet proceed. Do reclaim. */
	if (!compaction_suitable(zone, sc->order, sc->reclaim_idx))
		return false;

	/*
	 * Compaction is already possible, but it takes time to run and there
	 * are potentially other callers using the pages just freed. So proceed
	 * with reclaim to make a buffer of free pages available to give
	 * compaction a reasonable chance of completing and allocating the page.
	 * Note that we won't actually reclaim the whole buffer in one attempt
	 * as the target watermark in should_continue_reclaim() is lower. But if
	 * we are already above the high+gap watermark, don't reclaim at all.
	 */
	watermark = high_wmark_pages(zone) + compact_gap(sc->order);

	return zone_watermark_ok_safe(zone, 0, watermark, sc->reclaim_idx);
}

static void consider_reclaim_throttle(pg_data_t *pgdat, struct scan_control *sc)
{
	/*
	 * If reclaim is making progress greater than 12% efficiency then
	 * wake all the NOPROGRESS throttled tasks.
	 */
	if (sc->nr_reclaimed > (sc->nr_scanned >> 3)) {
		wait_queue_head_t *wqh;

		wqh = &pgdat->reclaim_wait[VMSCAN_THROTTLE_NOPROGRESS];
		if (waitqueue_active(wqh))
			wake_up(wqh);

		return;
	}

	/*
	 * Do not throttle kswapd or cgroup reclaim on NOPROGRESS as it will
	 * throttle on VMSCAN_THROTTLE_WRITEBACK if there are too many pages
	 * under writeback and marked for immediate reclaim at the tail of the
	 * LRU.
	 */
	if (current_is_kswapd() || cgroup_reclaim(sc))
		return;

	/* Throttle if making no progress at high prioities. */
	/* 如果sc优先级已经到1了，还没回收到内存，则节流休眠 */
	if (sc->priority == 1 && !sc->nr_reclaimed)
		reclaim_throttle(pgdat, VMSCAN_THROTTLE_NOPROGRESS);
}

/*
 * This is the direct reclaim path, for page-allocating processes.  We only
 * try to reclaim pages from zones which will satisfy the caller's allocation
 * request.
 *
 * If a zone is deemed to be full of pinned pages then just give it a light
 * scan then give up on it.
 */
/*
 * 直接回收核心流程
 *
 * 遍历zonelist所有符合条件的zone
 *	- 全局回收的特殊处理
 *		- 跳过的情况：gfp_mask不符合、可满足内存规整、重复回收node;
 *		- 尝试memcg软限制回收；
 *	- 调用**shrink_node**回收内存；
 */
static void shrink_zones(struct zonelist *zonelist, struct scan_control *sc)
{
	struct zoneref *z;
	struct zone *zone;
	unsigned long nr_soft_reclaimed;	/* memcg软限制回收的页面数量 */
	unsigned long nr_soft_scanned;		/* memcg软限制扫描的页面数量 */
	gfp_t orig_mask;
	pg_data_t *last_pgdat = NULL;	/* 上一个处理的node */
	pg_data_t *first_pgdat = NULL;	/* 第一个处理的node, 用于节流计算 */

	/*
	 * If the number of buffer_heads in the machine exceeds the maximum
	 * allowed level, force direct reclaim to scan the highmem zone as
	 * highmem pages could be pinning lowmem pages storing buffer_heads
	 */
        /*
         * 如果buffer_heads数量超过最大允许水平，
         * 强制直接回收扫描高端内存zone，因为高端内存页面
	 * 可能固定着存储buffer_heads的低端内存页面
         */
	orig_mask = sc->gfp_mask;
	if (buffer_heads_over_limit) {
		sc->gfp_mask |= __GFP_HIGHMEM;
		sc->reclaim_idx = gfp_zone(sc->gfp_mask);
	}

        /* 遍历zonelist中所有符合条件的zone */
	for_each_zone_zonelist_nodemask(zone, z, zonelist,
					sc->reclaim_idx, sc->nodemask) {
		/*
		 * Take care memory controller reclaiming has small influence
		 * to global LRU.
		 */
                /*
                 * 全局回收的特殊处理：内存控制器回收对全局LRU影响较小
                 */
		if (!cgroup_reclaim(sc)) {
			/* 如果当前zone不符合GFP_KERNEL | __GFP_HIGHMEM的内存分配要求, 则跳过 */
			if (!cpuset_zone_allowed(zone,
						 GFP_KERNEL | __GFP_HARDWALL))
				continue;

			/*
			 * If we already have plenty of memory free for
			 * compaction in this zone, don't free any more.
			 * Even though compaction is invoked for any
			 * non-zero order, only frequent costly order
			 * reclamation is disruptive enough to become a
			 * noticeable problem, like transparent huge
			 * page allocations.
			 */
			/*
			 * 如果使能了内存规整，并且当前内存分配请求的服务代价较大（order大于3）,
			 * 并且当前zone的空闲内存可以满足内存规整，则设置sc->compaction_ready
			 * 以便在退出后，可以根据这个来退出回收流程
			 */
			if (IS_ENABLED(CONFIG_COMPACTION) &&
			    sc->order > PAGE_ALLOC_COSTLY_ORDER &&
			    compaction_ready(zone, sc)) {
				sc->compaction_ready = true;
				continue;
			}

			/*
			 * Shrink each node in the zonelist once. If the
			 * zonelist is ordered by zone (not the default) then a
			 * node may be shrunk multiple times but in that case
			 * the user prefers lower zones being preserved.
			 */
			/*
			 * 一般每个node只被shrink一次(shrink_node)
			 * 如果zonelist按zone排序，则一个node可能被收缩多次
			 * 出现这种情况则跳过
			 */
			if (zone->zone_pgdat == last_pgdat)
				continue;

			/*
			 * This steals pages from memory cgroups over softlimit
			 * and returns the number of reclaimed pages and
			 * scanned pages. This works for global memory pressure
			 * and balancing, not for a memcg's limit.
			 */
			/*
			 * 全局回收下，尝试从memcg软限制中回收内存
			 *
                         * 从超过软限制的内存cgroup中窃取页面，返回回收和扫描的页面数。
                         * 这适用于全局内存压力和平衡，不适用于memcg的限制
			 */
			nr_soft_scanned = 0;
			nr_soft_reclaimed = memcg1_soft_limit_reclaim(zone->zone_pgdat,
								      sc->order, sc->gfp_mask,
								      &nr_soft_scanned);
			sc->nr_reclaimed += nr_soft_reclaimed;
			sc->nr_scanned += nr_soft_scanned;
			/* need some check for avoid more shrink_zone() */
		}

                /* 记录第一个pgdat用于节流计算 */
		if (!first_pgdat)
			first_pgdat = zone->zone_pgdat;

		/* See comment about same check for global reclaim above */
                /* 与上面全局回收相同的检查，避免重复处理同一个pgdat */
		if (zone->zone_pgdat == last_pgdat)
			continue;
		last_pgdat = zone->zone_pgdat;
                /*
		 * 核心回收函数：收缩特定节点
		 * 传入符合条件的zone对应的node的pgdat
		 * 和kswapd不一样，kswapd只回收自己所在的node
		 * 直接回收可能会遍历回收多个符合条件的node
		 */
		shrink_node(zone->zone_pgdat, sc);
	}

        /* 如果有处理过任何节点，考虑回收节流 */
	if (first_pgdat)
		consider_reclaim_throttle(first_pgdat, sc);

	/*
	 * Restore to original mask to avoid the impact on the caller if we
	 * promoted it to __GFP_HIGHMEM.
	 */
        /* 恢复原始掩码，避免如果我们将其提升为__GFP_HIGHMEM对调用者产生影响 */
	sc->gfp_mask = orig_mask;
}

static void snapshot_refaults(struct mem_cgroup *target_memcg, pg_data_t *pgdat)
{
	struct lruvec *target_lruvec;
	unsigned long refaults;

	if (lru_gen_enabled())
		return;

	target_lruvec = mem_cgroup_lruvec(target_memcg, pgdat);
	refaults = lruvec_page_state(target_lruvec, WORKINGSET_ACTIVATE_ANON);
	target_lruvec->refaults[WORKINGSET_ANON] = refaults;
	refaults = lruvec_page_state(target_lruvec, WORKINGSET_ACTIVATE_FILE);
	target_lruvec->refaults[WORKINGSET_FILE] = refaults;
}

/*
 * This is the main entry point to direct page reclaim.
 *
 * If a full scan of the inactive list fails to free enough memory then we
 * are "out of memory" and something needs to be killed.
 *
 * If the caller is !__GFP_FS then the probability of a failure is reasonably
 * high - the zone may be full of dirty or under-writeback pages, which this
 * caller can't do much about.  We kick the writeback threads and take explicit
 * naps in the hope that some of these pages can be written.  But if the
 * allocating task holds filesystem locks which prevent writeout this might not
 * work, and the allocation attempt will fail.
 *
 * returns:	0, if no pages reclaimed
 * 		else, the number of pages reclaimed
 */
/*
 * 直接回收的主要入口
 *
 * 1.逐步递减sc优先级，调用shrink_zones进行内存回收，直到回收满足以下三个条件之一
 *	- 回收到目标内存数量；
 *	- 回收的内存足以触发内存规整；
 *	- sc优先级递减到-1;
 * 2.回收后的处理：遍历所有zone，更新引用错误统计和清理状态
 * 3.有成功回收到页面，则退出；
 * 4.如果是因为要尝试内存规整而中止回收，返回1不触发OOM;
 * 5.回收不到内存，需要返回1重新尝试回收的情况
 *	- 没有完整遍历memcg，则尝试memcg完整遍历
 *	- 跳过回收inactive链表，则尝试回收inactive链表
 *	- 如果memcg使能了low内存保护（memcg使用内存不超过low，则不回收）,则关闭low内存报错，再尝试回收
 */
static unsigned long do_try_to_free_pages(struct zonelist *zonelist,
					  struct scan_control *sc)
{
	int initial_priority = sc->priority;
	pg_data_t *last_pgdat;
	struct zoneref *z;
	struct zone *zone;
retry:
	/*
	 * 统计任务关键事件的执行时间/延迟(cpu\io\memory)
	 * 可用于调整关键任务的优先级来优化场景性能
	 * 需要打开CONFIG_TASK_DELAY_ACCT
	 */
	delayacct_freepages_start();

        /* 如果是全局回收，统计分配停滞事件 */
	if (!cgroup_reclaim(sc))
		__count_zid_vm_events(ALLOCSTALL, sc->reclaim_idx, 1);

	/*
	 * 逐步递减sc优先级，调用shrink_zones进行内存回收，直到回收满足以下三个条件之一
	 *	- 回收到目标内存数量；
	 *	- 回收的内存足以触发内存规整；
	 *	- sc优先级递减到-1;
	 *
         * sc优先级越高（值越小），扫描越积极(扫描的页面数量越多)
	 */
	do {
                /* 非主动回收时，记录内存压力事件 */
		if (!sc->proactive)
			vmpressure_prio(sc->gfp_mask, sc->target_mem_cgroup,
					sc->priority);
		sc->nr_scanned = 0;
                /* 核心函数：扫描并回收各个zone的内存 */
		 shrink_zones(zonelist, sc);

		/* 如果回收到符合需求的内存数量，则退出 */
		if (sc->nr_reclaimed >= sc->nr_to_reclaim)
			break;

		/*
		 * shrink_zones中判断有zone可以开始做内存规整时，会设置sc->compaction_ready
		 * 后续退出回收后，会尝试compact后再尝试分配内存
		 * __alloc_pages_slowpath-->__alloc_pages_direct_compact
		 */
		if (sc->compaction_ready)
			break;

		/*
		 * If we're getting trouble reclaiming, start doing
		 * writepage even in laptop mode.
		 */
		/*
		 * 当sc优先级降到10以下，还没完成回收，则开始允许回写
		 */
		if (sc->priority < DEF_PRIORITY - 2)
			sc->may_writepage = 1;
	} while (--sc->priority >= 0);

        /*
         * 回收后的处理：遍历所有zone，更新引用错误统计和清理状态
         * 每个pgdat只处理一次，避免重复操作
         */
	last_pgdat = NULL;
	for_each_zone_zonelist_nodemask(zone, z, zonelist, sc->reclaim_idx,
					sc->nodemask) {
		if (zone->zone_pgdat == last_pgdat)
			continue;
		last_pgdat = zone->zone_pgdat;

                /* 快照引用错误统计，用于后续工作集检测 */
		snapshot_refaults(sc->target_mem_cgroup, zone->zone_pgdat);

                /* 如果是cgroup回收，清理对应的拥塞标志 */
		if (cgroup_reclaim(sc)) {
			struct lruvec *lruvec;

			lruvec = mem_cgroup_lruvec(sc->target_mem_cgroup,
						   zone->zone_pgdat);
			clear_bit(LRUVEC_CGROUP_CONGESTED, &lruvec->flags);
		}
	}

	delayacct_freepages_end();

        /* 如果有回收到页面，返回回收数量 */
	if (sc->nr_reclaimed)
		return sc->nr_reclaimed;

	/* Aborted reclaim to try compaction? don't OOM, then */
        /*
	 * 如果是因为要尝试内存规整而中止回收，返回1不触发OOMs
	 * 返回1，使得在返回到__alloc_pages_slowpath时还有retry的可能，不至于直接oom
	 */
	if (sc->compaction_ready)
		return 1;

	/*
	 * 下面是回收不到内存，需要返回重新尝试回收的情况
	 */

	/*
	 * In most cases, direct reclaimers can do partial walks
	 * through the cgroup tree to meet the reclaim goal while
	 * keeping latency low. Since the iterator state is shared
	 * among all direct reclaim invocations (to retain fairness
	 * among cgroups), though, high concurrency can result in
	 * individual threads not seeing enough cgroups to make
	 * meaningful forward progress. Avoid false OOMs in this case.
	 */
	/*
	 * 重试1: 没有完整遍历memcg，则尝试memcg完整遍历
	 */
	if (!sc->memcg_full_walk) {
		sc->priority = initial_priority;
		sc->memcg_full_walk = 1;
		goto retry;
	}

	/*
	 * We make inactive:active ratio decisions based on the node's
	 * composition of memory, but a restrictive reclaim_idx or a
	 * memory.low cgroup setting can exempt large amounts of
	 * memory from reclaim. Neither of which are very common, so
	 * instead of doing costly eligibility calculations of the
	 * entire cgroup subtree up front, we assume the estimates are
	 * good, and retry with forcible deactivation if that fails.
	 */
	/*
	 * 重试2：不回收inactive链表，则尝试回收inactive链表
	 */
	if (sc->skipped_deactivate) {
		sc->priority = initial_priority;
		sc->force_deactivate = 1;
		sc->skipped_deactivate = 0;
		goto retry;
	}

	/* Untapped cgroup reserves?  Don't OOM, retry. */
	/*
	 * 重试3：如果memcg使能了low内存保护（memcg使用内存不超过low，则不回收）
	 *        则关闭low内存报错，再尝试回收
	 */
	if (sc->memcg_low_skipped) {
		sc->priority = initial_priority;
		sc->force_deactivate = 0;
		sc->memcg_low_reclaim = 1;
		sc->memcg_low_skipped = 0;
		goto retry;
	}

	/*
	 * 连直接回收都回收不到页面，则返回0，触发oom
	 */
	return 0;
}

/*
 * 检查是否允许直接内存回收。该函数评估PFMEMALLOC保留内存的水印状态，
 * 决定是否对直接回收进程进行节流。
 *
 * 返回true表示允许直接回收，false表示需要节流。
 */
static bool allow_direct_reclaim(pg_data_t *pgdat)
{
	struct zone *zone;
	unsigned long pfmemalloc_reserve = 0;
	unsigned long free_pages = 0;
	int i;
	bool wmark_ok;

        /*
         * 如果kswapd已经多次回收失败，认为该节点无法通过后台回收改善，
         * 允许直接回收，避免完全阻塞内存分配。
         *
         * 在极端内存压力下，即使PFMEMALLOC不足也要尝试回收，
         * 否则系统可能完全停滞。
         */
	if (pgdat->kswapd_failures >= MAX_RECLAIM_RETRIES)
		return true;

        /*
         * 遍历NORMAL及以下的zone（ZONE_DMA, ZONE_DMA32, ZONE_NORMAL）
         * HIGHMEM区域不参与PFMEMALLOC计算，因为网络缓冲区等关键分配
         * 通常需要GFP_KERNEL标志，无法使用HIGHMEM内存。
         */
	for (i = 0; i <= ZONE_NORMAL; i++) {
		zone = &pgdat->node_zones[i];
		/* 跳过未管理的zone */
		if (!managed_zone(zone))
			continue;

		/* 跳过没有内存可以回收的zone */
		if (!zone_reclaimable_pages(zone))
			continue;

		/* 累加min水位线作为pfmemalloc需求 */
		pfmemalloc_reserve += min_wmark_pages(zone);
                /* 累加当前空闲页面数量（使用快照避免并发问题） */
		free_pages += zone_page_state_snapshot(zone, NR_FREE_PAGES);
	}

	/* If there are no reserves (unexpected config) then do not throttle */
        /* 如果没有配置保留内存/min水位线（异常配置），则始终允许直接回收 */
	if (!pfmemalloc_reserve)
		return true;
	/*
         * 核心检查：当前空闲内存是否大于PFMEMALLOC保留需求的一半
         * 这是保守的水位线检查，确保有足够的保留内存供关键系统功能使用
	 *
	 * 如果有足够多的空闲内存可以满足PFMEMALLOC，则可以进行直接回收
         */
	wmark_ok = free_pages > pfmemalloc_reserve / 2;

	/* kswapd must be awake if processes are being throttled */
	/*
	 * 如果上面的水位线检查失败(空闲内存不满足PFMEMALLOC)，
	 * 并且kswapd在休眠，则唤醒kswapd进行后台回收
	 *
	 * 这是确保系统不会因为PFMEMALLOC不足而死锁的关键机制
	 */
	if (!wmark_ok && waitqueue_active(&pgdat->kswapd_wait)) {
		if (READ_ONCE(pgdat->kswapd_highest_zoneidx) > ZONE_NORMAL)
			WRITE_ONCE(pgdat->kswapd_highest_zoneidx, ZONE_NORMAL);

		wake_up_interruptible(&pgdat->kswapd_wait);
	}

	return wmark_ok;
}

/*
 * Throttle direct reclaimers if backing storage is backed by the network
 * and the PFMEMALLOC reserve for the preferred node is getting dangerously
 * depleted. kswapd will continue to make progress and wake the processes
 * when the low watermark is reached.
 *
 * Returns true if a fatal signal was delivered during throttling. If this
 * happens, the page allocator should not consider triggering the OOM killer.
 */
static bool throttle_direct_reclaim(gfp_t gfp_mask, struct zonelist *zonelist,
					nodemask_t *nodemask)
{
	struct zoneref *z;
	struct zone *zone;
	pg_data_t *pgdat = NULL;

	/*
	 * Kernel threads should not be throttled as they may be indirectly
	 * responsible for cleaning pages necessary for reclaim to make forward
	 * progress. kjournald for example may enter direct reclaim while
	 * committing a transaction where throttling it could forcing other
	 * processes to block on log_wait_commit().
	 */
	/*
	 * 内核线程不节流
	 * 内核线程通常执行关键的系统任务，节流它们可能导致系统死锁或性能问题
	 */
	if (current->flags & PF_KTHREAD)
		goto out;

	/*
	 * If a fatal signal is pending, this process should not throttle.
	 * It should return quickly so it can exit and free its memory
	 */
	if (fatal_signal_pending(current))
		goto out;

	/*
	 * Check if the pfmemalloc reserves are ok by finding the first node
	 * with a usable ZONE_NORMAL or lower zone. The expectation is that
	 * GFP_KERNEL will be required for allocating network buffers when
	 * swapping over the network so ZONE_HIGHMEM is unusable.
	 *
	 * Throttling is based on the first usable node and throttled processes
	 * wait on a queue until kswapd makes progress and wakes them. There
	 * is an affinity then between processes waking up and where reclaim
	 * progress has been made assuming the process wakes on the same node.
	 * More importantly, processes running on remote nodes will not compete
	 * for remote pfmemalloc reserves and processes on different nodes
	 * should make reasonable progress.
	 */
        /*
         * 通过找到第一个具有可用ZONE_NORMAL或更低zone的节点来检查pfmemalloc保留内存是否正常。
         * 预期是在通过网络进行交换时，分配网络缓冲区需要GFP_KERNEL标志，因此ZONE_HIGHMEM不可用。
         *
         * 节流基于第一个可用节点，被节流的进程在队列上等待，直到kswapd取得进展并唤醒它们。
         * 假设进程在同一节点上唤醒，那么在唤醒进程和回收进展之间存在亲和性。
         * 更重要的是，在远程节点上运行的进程不会竞争远程pfmemalloc保留内存，
         * 不同节点上的进程应该取得合理的进展。
         *
         * 底层原理：PFMEMALLOC是用于网络存储等关键功能的保留内存，当这些内存不足时，
         * 需要让直接回收等待，避免影响系统关键功能。
         */
	for_each_zone_zonelist_nodemask(zone, z, zonelist,
					gfp_zone(gfp_mask), nodemask) {
		if (zone_idx(zone) > ZONE_NORMAL)
			continue;

		/* Throttle based on the first usable node */
		pgdat = zone->zone_pgdat;
		if (allow_direct_reclaim(pgdat))
			goto out;
		break;
	}

	/* If no zone was usable by the allocation flags then do not throttle */
	if (!pgdat)
		goto out;

	/* Account for the throttling */
	count_vm_event(PGSCAN_DIRECT_THROTTLE);

	/*
	 * If the caller cannot enter the filesystem, it's possible that it
	 * is due to the caller holding an FS lock or performing a journal
	 * transaction in the case of a filesystem like ext[3|4]. In this case,
	 * it is not safe to block on pfmemalloc_wait as kswapd could be
	 * blocked waiting on the same lock. Instead, throttle for up to a
	 * second before continuing.
	 */
	 /*
         * 如果调用者不能进入文件系统，可能是因为调用者持有FS锁或在执行日志事务
         *（对于ext[3|4]等文件系统）。这种情况下，在pfmemalloc_wait上阻塞是不安全的，
         * 因为kswapd可能被阻塞等待同一个锁。相反，在继续之前节流最多一秒。
         *
         * __GFP_FS标志表示可以调用文件系统相关函数。如果没有这个标志，
         * 说明调用者可能持有文件系统锁，长时间阻塞可能导致kswapd也阻塞在同一个锁上，
         * 形成死锁。因此使用超时等待而不是无限等待。
         */

	if (!(gfp_mask & __GFP_FS))
		wait_event_interruptible_timeout(pgdat->pfmemalloc_wait,
			allow_direct_reclaim(pgdat), HZ);
	else
		/* Throttle until kswapd wakes the process */
		/* 节流直到被kswapd唤醒 */
		wait_event_killable(zone->zone_pgdat->pfmemalloc_wait,
			allow_direct_reclaim(pgdat));

	if (fatal_signal_pending(current))
		return true;

out:
	return false;
}

unsigned long try_to_free_pages(struct zonelist *zonelist, int order,
				gfp_t gfp_mask, nodemask_t *nodemask)
{
	unsigned long nr_reclaimed;
	struct scan_control sc = {
		.nr_to_reclaim = SWAP_CLUSTER_MAX,
		.gfp_mask = current_gfp_context(gfp_mask),
		.reclaim_idx = gfp_zone(gfp_mask),
		.order = order,
		.nodemask = nodemask,
		.priority = DEF_PRIORITY,
		.may_writepage = !laptop_mode,
		.may_unmap = 1,
		.may_swap = 1,
	};

	/*
	 * scan_control uses s8 fields for order, priority, and reclaim_idx.
	 * Confirm they are large enough for max values.
	 */
	BUILD_BUG_ON(MAX_PAGE_ORDER >= S8_MAX);
	BUILD_BUG_ON(DEF_PRIORITY > S8_MAX);
	BUILD_BUG_ON(MAX_NR_ZONES > S8_MAX);

	/*
	 * Do not enter reclaim if fatal signal was delivered while throttled.
	 * 1 is returned so that the page allocator does not OOM kill at this
	 * point.
	 */
	/*
	 * 判断是否需要节流，如果需要增加入等待队(pfmemalloc_wait)等待，直到被kswapd唤醒
	 */
	if (throttle_direct_reclaim(sc.gfp_mask, zonelist, nodemask))
		return 1;

	set_task_reclaim_state(current, &sc.reclaim_state);
	trace_mm_vmscan_direct_reclaim_begin(order, sc.gfp_mask);

	nr_reclaimed = do_try_to_free_pages(zonelist, &sc);

	trace_mm_vmscan_direct_reclaim_end(nr_reclaimed);
	set_task_reclaim_state(current, NULL);

	return nr_reclaimed;
}

#ifdef CONFIG_MEMCG

/* Only used by soft limit reclaim. Do not reuse for anything else. */
unsigned long mem_cgroup_shrink_node(struct mem_cgroup *memcg,
						gfp_t gfp_mask, bool noswap,
						pg_data_t *pgdat,
						unsigned long *nr_scanned)
{
	struct lruvec *lruvec = mem_cgroup_lruvec(memcg, pgdat);
	struct scan_control sc = {
		.nr_to_reclaim = SWAP_CLUSTER_MAX,
		.target_mem_cgroup = memcg,
		.may_writepage = !laptop_mode,
		.may_unmap = 1,
		.reclaim_idx = MAX_NR_ZONES - 1,
		.may_swap = !noswap,
	};

	WARN_ON_ONCE(!current->reclaim_state);

	sc.gfp_mask = (gfp_mask & GFP_RECLAIM_MASK) |
			(GFP_HIGHUSER_MOVABLE & ~GFP_RECLAIM_MASK);

	trace_mm_vmscan_memcg_softlimit_reclaim_begin(sc.order,
						      sc.gfp_mask);

	/*
	 * NOTE: Although we can get the priority field, using it
	 * here is not a good idea, since it limits the pages we can scan.
	 * if we don't reclaim here, the shrink_node from balance_pgdat
	 * will pick up pages from other mem cgroup's as well. We hack
	 * the priority and make it zero.
	 */
	shrink_lruvec(lruvec, &sc);

	trace_mm_vmscan_memcg_softlimit_reclaim_end(sc.nr_reclaimed);

	*nr_scanned = sc.nr_scanned;

	return sc.nr_reclaimed;
}

unsigned long try_to_free_mem_cgroup_pages(struct mem_cgroup *memcg,
					   unsigned long nr_pages,
					   gfp_t gfp_mask,
					   unsigned int reclaim_options,
					   int *swappiness)
{
	unsigned long nr_reclaimed;
	unsigned int noreclaim_flag;
	struct scan_control sc = {
		.nr_to_reclaim = max(nr_pages, SWAP_CLUSTER_MAX),
		.proactive_swappiness = swappiness,
		.gfp_mask = (current_gfp_context(gfp_mask) & GFP_RECLAIM_MASK) |
				(GFP_HIGHUSER_MOVABLE & ~GFP_RECLAIM_MASK),
		.reclaim_idx = MAX_NR_ZONES - 1,
		.target_mem_cgroup = memcg,
		.priority = DEF_PRIORITY,
		.may_writepage = !laptop_mode,
		.may_unmap = 1,
		.may_swap = !!(reclaim_options & MEMCG_RECLAIM_MAY_SWAP),
		.proactive = !!(reclaim_options & MEMCG_RECLAIM_PROACTIVE),
	};
	/*
	 * Traverse the ZONELIST_FALLBACK zonelist of the current node to put
	 * equal pressure on all the nodes. This is based on the assumption that
	 * the reclaim does not bail out early.
	 */
	struct zonelist *zonelist = node_zonelist(numa_node_id(), sc.gfp_mask);

	set_task_reclaim_state(current, &sc.reclaim_state);
	trace_mm_vmscan_memcg_reclaim_begin(0, sc.gfp_mask);
	noreclaim_flag = memalloc_noreclaim_save();

	nr_reclaimed = do_try_to_free_pages(zonelist, &sc);

	memalloc_noreclaim_restore(noreclaim_flag);
	trace_mm_vmscan_memcg_reclaim_end(nr_reclaimed);
	set_task_reclaim_state(current, NULL);

	return nr_reclaimed;
}
#endif

/*
 * kswapd老化流程
 *	1.如果使能了MGLRU，则走MGLRU页面老化评估流程；(包含匿名页面和文件页面)
 *	2.如果没有使能MGLRU，则走传统LRU老化；(只老化匿名页)
 *		- 使能了swap或者当前node支持降级才可以老化；
 *		- inactive还不是low水平，也不用老化；
 *		- 遍历memcg, 调用shrink_active_list(LRU_ACTIVE_ANON)老化匿名页；
 */
static void kswapd_age_node(struct pglist_data *pgdat, struct scan_control *sc)
{
	struct mem_cgroup *memcg;
	struct lruvec *lruvec;

	/*
	 * 如果使能了MGLRU，则走MGLRU的老化路径
	 * **lru_gen_age_node并不是老化页面**
	 * 而是评估页面的新老程度，看哪些页面可以回收
	 *
	 * lru_gen_age_node 会根据引用信息推进各代的边界，从而更精准地区分冷热页，
	 * 这比传统active/inactive LRU更细粒度，有利于回收命中冷页、留住热页。
	 */
	if (lru_gen_enabled()) {
		lru_gen_age_node(pgdat, sc);
		return;
	}

	/* 下面的传统的老化路径（非MGLRU）*/
	/*
	 * 匿名页面可以老化的条件
	 *	- 使能了swap;
	 *	- 当前node支持内存降级(demotion);
	 *
	 * 如果匿名页面不可以老化，则直接退出
	 */
	if (!can_age_anon_pages(pgdat, sc))
		return;

	/* 获取当前node 的lruvec */
	lruvec = mem_cgroup_lruvec(NULL, pgdat);
	/* 如果inactive还不是low水平, 则不需要老化，直接返回 */
	if (!inactive_is_low(lruvec, LRU_INACTIVE_ANON))
		return;

	/*
	 * 从root memcg开始，遍历所有memcg
	 * 走memcg分层，按层遍历每个memcg在该node上的lruvec，对匿名活跃队列做“收缩”：
	 * - shrink_active_list会选择性地将部分活跃匿名页降为不活跃，或触发引用检测，
	 *   这是“背景老化”的关键动作，旨在建立足够规模的冷页池（inactive_anon）。
	 * - 仅在inactive_anon不足时才进行，以避免过度打扰活跃工作集。
	 */
	memcg = mem_cgroup_iter(NULL, NULL, NULL);
	do {
		/* 根据memcg和node, 获取对应的lruvec */
		lruvec = mem_cgroup_lruvec(memcg, pgdat);
		shrink_active_list(SWAP_CLUSTER_MAX, lruvec,
				   sc, LRU_ACTIVE_ANON);
		memcg = mem_cgroup_iter(NULL, memcg, NULL);
	} while (memcg);
}

static bool pgdat_watermark_boosted(pg_data_t *pgdat, int highest_zoneidx)
{
	int i;
	struct zone *zone;

	/*
	 * Check for watermark boosts top-down as the higher zones
	 * are more likely to be boosted. Both watermarks and boosts
	 * should not be checked at the same time as reclaim would
	 * start prematurely when there is no boosting and a lower
	 * zone is balanced.
	 */
	for (i = highest_zoneidx; i >= 0; i--) {
		zone = pgdat->node_zones + i;
		if (!managed_zone(zone))
			continue;

		if (zone->watermark_boost)
			return true;
	}

	return false;
}

/*
 * Returns true if there is an eligible zone balanced for the request order
 * and highest_zoneidx
 */
/*
 * 判断当前node是否已经balanced
 * 也即是当前node下的zone的空闲内存是否满足水位线，是否可以让kswapd休眠
 *
 * 没平衡的条件：
 *	- zone free内存低于高水位线+lowmem_reserve, not balanced
 *	- zone free内存高于高水位线+lowmem_reserve的情况下
 *		- high-order申请，这个high-order及更高order的buddy空闲链表中都没有空闲块，not balanced
 *
 * 平衡的条件：
 *	- 没有可管理的zone, balanced
 *	- zone free内存高于高水位线+lowmem_reserve的情况下
 *		- 0-order申请, balanced
 *		- high-order申请，至少在这个high-order及更高order的buddy空闲链表中要有一个空闲块, balanced
 *
 *	只要有一个zone达到上面的条件，就算当前node平衡了
 */

static bool pgdat_balanced(pg_data_t *pgdat, int order, int highest_zoneidx)
{
	int i;
	unsigned long mark = -1;
	struct zone *zone;

	/*
	 * Check watermarks bottom-up as lower zones are more likely to
	 * meet watermarks.
	 */
	/*
	 * 从高的zone开始检查水位线
	 * 低端zone(DMA)一般有更严格的水位线要求
	 */
	for (i = 0; i <= highest_zoneidx; i++) {
		zone = pgdat->node_zones + i;

		if (!managed_zone(zone))
			continue;

		/*
                 * 根据NUMA平衡模式选择适当的水位线：
                 * - 如果启用内存分层优化，使用promotion水位线（更积极）
                 * - 否则使用高水位线（标准检查）
                 * 
                 * NUMA_BALANCING_MEMORY_TIERING模式优化内存分层访问，
                 * 允许更积极的内存迁移策略。
                 */
		if (sysctl_numa_balancing_mode & NUMA_BALANCING_MEMORY_TIERING)
			mark = promo_wmark_pages(zone);
		else
			mark = high_wmark_pages(zone);
		if (zone_watermark_ok_safe(zone, order, mark, highest_zoneidx))
			return true;
	}

	/*
	 * If a node has no managed zone within highest_zoneidx, it does not
	 * need balancing by definition. This can happen if a zone-restricted
	 * allocation tries to wake a remote kswapd.
	 */
	if (mark == -1)
		return true;

	return false;
}

/* Clear pgdat state for congested, dirty or under writeback. */
static void clear_pgdat_congested(pg_data_t *pgdat)
{
	struct lruvec *lruvec = mem_cgroup_lruvec(NULL, pgdat);

	clear_bit(LRUVEC_NODE_CONGESTED, &lruvec->flags);
	clear_bit(LRUVEC_CGROUP_CONGESTED, &lruvec->flags);
	clear_bit(PGDAT_DIRTY, &pgdat->flags);
	clear_bit(PGDAT_WRITEBACK, &pgdat->flags);
}

/*
 * Prepare kswapd for sleeping. This verifies that there are no processes
 * waiting in throttle_direct_reclaim() and that watermarks have been met.
 *
 * Returns true if kswapd is ready to sleep
 */
/*
 * 判断kswapd能否进入休眠, 可以休眠则返回true
 *	1.无论如何，先把在直接回收中被节流休眠的任务唤醒；
 *	2.可以休眠的条件：
 *		- kswapd无法在当前node回收到内存；
 *		- 当前node的空闲内存满足水位线要求(balanced)；
 */
static bool prepare_kswapd_sleep(pg_data_t *pgdat, int order,
				int highest_zoneidx)
{
	/*
	 * The throttled processes are normally woken up in balance_pgdat() as
	 * soon as allow_direct_reclaim() is true. But there is a potential
	 * race between when kswapd checks the watermarks and a process gets
	 * throttled. There is also a potential race if processes get
	 * throttled, kswapd wakes, a large process exits thereby balancing the
	 * zones, which causes kswapd to exit balance_pgdat() before reaching
	 * the wake up checks. If kswapd is going to sleep, no process should
	 * be sleeping on pfmemalloc_wait, so wake them now if necessary. If
	 * the wake up is premature, processes will wake kswapd and get
	 * throttled again. The difference from wake ups in balance_pgdat() is
	 * that here we are under prepare_to_wait().
	 */
	/*
	 * ds:
         * 被节流的进程通常会在balance_pgdat()中一旦allow_direct_reclaim()为true时被唤醒。
         * 但是在kswapd检查水位线和进程被节流之间存在潜在的竞争条件。
         * 还有一个潜在的竞争：如果进程被节流，kswapd唤醒，然后一个大型进程退出从而平衡了内存域，
         * 这会导致kswapd在达到唤醒检查之前就退出balance_pgdat()。
         * 如果kswapd即将睡眠，不应该有进程在pfmemalloc_wait上睡眠，
         * 所以如果有必要现在就唤醒它们。如果唤醒是过早的，进程会再次唤醒kswapd并重新被节流。
         * 与balance_pgdat()中的唤醒不同的是，这里我们处于prepare_to_wait()之下。
         * 
         * 底层原理：防止kswapd睡眠时还有进程在等待PFMEMALLOC内存，避免死锁。
	 */
	/*
	 * 如果pfmemalloc_wait等待队列不为空，也就是在throttle_direct_reclaim()中等待，
	 * 则在kswapd进入休眠之前，先唤醒这些等待的进程
	 * 就算此时空闲内存还不满足pfmemalloc，打不了再对唤醒的直接回收任务进行重新节流
	 */
	if (waitqueue_active(&pgdat->pfmemalloc_wait))
		wake_up_all(&pgdat->pfmemalloc_wait);

	/* Hopeless node, leave it to direct reclaim */
	/*
	 * kswapd无法在这个node上回收内存，则kswapd休眠，让直接回收来解决
	 */
	if (pgdat->kswapd_failures >= MAX_RECLAIM_RETRIES)
		return true;
        /*
         * 检查节点是否已经平衡：所有相关zone的水印都满足要求
         * pgdat_balanced会检查order要求的内存连续性和highest_zoneidx指定的最高zone
	 *
	 * 如果当前节点已经balanced、空闲内存满足水位线，则kswapd可以休眠
         */
	if (pgdat_balanced(pgdat, order, highest_zoneidx)) {
		clear_pgdat_congested(pgdat);
		return true;
	}

	return false;
}

/*
 * kswapd shrinks a node of pages that are at or below the highest usable
 * zone that is currently unbalanced.
 *
 * Returns true if kswapd scanned at least the requested number of pages to
 * reclaim or if the lack of progress was due to pages under writeback.
 * This is used to determine if the scanning priority needs to be raised.
 */
static bool kswapd_shrink_node(pg_data_t *pgdat,
			       struct scan_control *sc)
{
	struct zone *zone;
	int z;
	/* 获取已经回收到的页面数量 */
	unsigned long nr_reclaimed = sc->nr_reclaimed;

	/* Reclaim a number of pages proportional to the number of zones */
	/*
	 * 重新计算需要回收的页面数量
	 * 回收页面数量等于目标zone的high watermark和SWAP_CLUSTER_MAX中的最大值的累加
	 */
	sc->nr_to_reclaim = 0;
	for (z = 0; z <= sc->reclaim_idx; z++) {
		zone = pgdat->node_zones + z;
		if (!managed_zone(zone))
			continue;

		sc->nr_to_reclaim += max(high_wmark_pages(zone), SWAP_CLUSTER_MAX);
	}

	/*
	 * Historically care was taken to put equal pressure on all zones but
	 * now pressure is applied based on node LRU order.
	 */
	shrink_node(pgdat, sc);

	/*
	 * Fragmentation may mean that the system cannot be rebalanced for
	 * high-order allocations. If twice the allocation size has been
	 * reclaimed then recheck watermarks only at order-0 to prevent
	 * excessive reclaim. Assume that a process requested a high-order
	 * can direct reclaim/compact.
	 */
	if (sc->order && sc->nr_reclaimed >= compact_gap(sc->order))
		sc->order = 0;

	/* account for progress from mm_account_reclaimed_pages() */
	return max(sc->nr_scanned, sc->nr_reclaimed - nr_reclaimed) >= sc->nr_to_reclaim;
}

/* Page allocator PCP high watermark is lowered if reclaim is active. */
static inline void
update_reclaim_active(pg_data_t *pgdat, int highest_zoneidx, bool active)
{
	int i;
	struct zone *zone;

	for (i = 0; i <= highest_zoneidx; i++) {
		zone = pgdat->node_zones + i;

		if (!managed_zone(zone))
			continue;

		if (active)
			set_bit(ZONE_RECLAIM_ACTIVE, &zone->flags);
		else
			clear_bit(ZONE_RECLAIM_ACTIVE, &zone->flags);
	}
}

/*
 * 设置zone的活跃回收状态，会影响PCP, 也就是每个cpu的页面缓存，用于快速分配单页
 *
 * 设置zone为活跃回收状态时，会降低PCP的水位线，也就是减少per-cpu页面缓存
 * 这样可以回收更多页面
 */
static inline void
set_reclaim_active(pg_data_t *pgdat, int highest_zoneidx)
{
	update_reclaim_active(pgdat, highest_zoneidx, true);
}

static inline void
clear_reclaim_active(pg_data_t *pgdat, int highest_zoneidx)
{
	update_reclaim_active(pgdat, highest_zoneidx, false);
}

/*
 * For kswapd, balance_pgdat() will reclaim pages across a node from zones
 * that are eligible for use by the caller until at least one zone is
 * balanced.
 *
 * Returns the order kswapd finished reclaiming at.
 *
 * kswapd scans the zones in the highmem->normal->dma direction.  It skips
 * zones which have free_pages > high_wmark_pages(zone), but once a zone is
 * found to have free_pages <= high_wmark_pages(zone), any page in that zone
 * or lower is eligible for reclaim until at least one usable zone is
 * balanced.
 */
/*
 * 常规回收和boost回收的区别
 *
 * 常规回收
	- 允许writeback和swap;
	- 不限制扫描优先级/页面扫描数量；
	- 回收没进展(回收页面为0)，还继续回收；
	- 结束后核销zone->watermark_boost 并唤醒 kcompactd 做页面规整；

 * boost回收
	- 不允许writeback和swap;
	- 限制扫描优先级/页面扫描数量；
	- 回收没进展(回收页面为0)，则立刻结束；
	- 运行完整的shrinker/LRU老化，直到达到水位或用尽优先级。

 * 总结
	- 常规回收比boost回收力度大, 更激进；
	- 常规回收, 在memory node失衡（低于水位）时恢复整体平衡，保障系统持续分配能力;
	- boost 回收，为"接近水位/高阶分配受阻/碎片化风险"做短期抬水位, 目标是快速"挪出一点空闲"，避免次优的 I/O。
 *
 */
/*
 * balance_pgdat会从符合使用者申请条件的zone回收内存, 直到至少有一个zone达到balanced(空闲内存满足水位线)
 */
static int balance_pgdat(pg_data_t *pgdat, int order, int highest_zoneidx)
{
	int i;
	unsigned long nr_soft_reclaimed;	// 软限制回收的页面数
	unsigned long nr_soft_scanned;		// 软限制扫描的页面数
	unsigned long pflags;
	unsigned long nr_boost_reclaim;
	unsigned long zone_boosts[MAX_NR_ZONES] = { 0, };
	bool boosted;
	struct zone *zone;
	struct scan_control sc = {
		.gfp_mask = GFP_KERNEL,
		.order = order,
		.may_unmap = 1,
	};

	set_task_reclaim_state(current, &sc.reclaim_state);
	psi_memstall_enter(&pflags);
	__fs_reclaim_acquire(_THIS_IP_);

	count_vm_event(PAGEOUTRUN);

	/*
	 * Account for the reclaim boost. Note that the zone boost is left in
	 * place so that parallel allocations that are near the watermark will
	 * stall or direct reclaim until kswapd is finished.
	 */
	nr_boost_reclaim = 0;
	for (i = 0; i <= highest_zoneidx; i++) {
		zone = pgdat->node_zones + i;
		if (!managed_zone(zone))
			continue;

		/*
		 * 获取每个zone的watermark__boost
		 * watermark_boost_factor，用于临时抬高水位线
		 */
		nr_boost_reclaim += zone->watermark_boost;
		zone_boosts[i] = zone->watermark_boost;
	}
	boosted = nr_boost_reclaim;

restart:
	/*
	 * 设置zone回收活跃状态，以降低PCP页面缓存
	 * 从而回收到更多的页面
	 */
	set_reclaim_active(pgdat, highest_zoneidx);
	/*
	 * 初始化优先级
	 * 每次扫描的页面数为tatal_size>>priority, DEF_PRIORITY为12
	 * 随着内存回收的深入(回收难度变大、回收的内存不满足要求)，
	 * priority会递减，扫描的页面数量会越多。
	 */
	sc.priority = DEF_PRIORITY;
	do {
		unsigned long nr_reclaimed = sc.nr_reclaimed;
		bool raise_priority = true;
		bool balanced;
		bool ret;
		bool was_frozen;

		sc.reclaim_idx = highest_zoneidx;

		/*
		 * If the number of buffer_heads exceeds the maximum allowed
		 * then consider reclaiming from all zones. This has a dual
		 * purpose -- on 64-bit systems it is expected that
		 * buffer_heads are stripped during active rotation. On 32-bit
		 * systems, highmem pages can pin lowmem memory and shrinking
		 * buffers can relieve lowmem pressure. Reclaim may still not
		 * go ahead if all eligible zones for the original allocation
		 * request are balanced to avoid excessive reclaim from kswapd.
		 */
		/*
		 * 如果buffer_heads数量超过最大允许值，则考虑从所有区域回收
		 * 这有双重目的——在64位系统上，预期buffer_heads在活跃旋转期间被剥离
		 * 在32位系统上，高端内存页面可以固定低端内存，收缩缓冲区可以缓解低端内存压力
		 * 如果原始分配请求的所有合格区域都已平衡，回收可能仍然不会进行，
		 * 以避免kswapd过度回收
		 */
		if (buffer_heads_over_limit) {
			for (i = MAX_NR_ZONES - 1; i >= 0; i--) {
				zone = pgdat->node_zones + i;
				if (!managed_zone(zone))
					continue;

				sc.reclaim_idx = i;	// 扩展到所有zone
				break;
			}
		}

		/*
		 * If the pgdat is imbalanced then ignore boosting and preserve
		 * the watermarks for a later time and restart. Note that the
		 * zone watermarks will be still reset at the end of balancing
		 * on the grounds that the normal reclaim should be enough to
		 * re-evaluate if boosting is required when kswapd next wakes.
		 */
		balanced = pgdat_balanced(pgdat, sc.order, highest_zoneidx);
		/*
		 * 当内存node没达到平衡时，优先使用常规回收，尽快将node拉回水位线，而不是boost回收
		 * 将nr_boost_reclaim置0, 再重新开始
		 * 这次kswap都不使用boost回收，kswapd下次唤醒有机会使用boost
		 */
		if (!balanced && nr_boost_reclaim) {
			nr_boost_reclaim = 0;
			goto restart;
		}

		/*
		 * If boosting is not active then only reclaim if there are no
		 * eligible zones. Note that sc.reclaim_idx is not used as
		 * buffer_heads_over_limit may have adjusted it.
		 */
		/*
		 * 如果没有激活boost回收，而且node已经平衡了，这直接退出；
		 * 这避免不必要的背景回收，降低写放大、I/O抖动和CPU开销;(待理解)
		 */
		if (!nr_boost_reclaim && balanced)
			goto out;

		/* Limit the priority of boosting to avoid reclaim writeback */
		/*
		 * **boost回收，限制扫描优先级，减少页面扫描数量**
		 *
		 * 限制boost回收，降低优先级的速度，避免进入writeback阶段
		 * （how？我以为是减少页面扫描数量）
		 */
		if (nr_boost_reclaim && sc.priority == DEF_PRIORITY - 2)
			raise_priority = false;

		/*
		 * Do not writeback or swap pages for boosted reclaim. The
		 * intent is to relieve pressure not issue sub-optimal IO
		 * from reclaim context. If no pages are reclaimed, the
		 * reclaim will be aborted.
		 */
		/*
		 * **boost回收，不允许writeback和swap**
		 *
		 * boost回收的目标是“快速腾挪空闲”，避免在回收路径上发起昂贵/次优的I/O：
		 * - 禁止写回（may_writepage=0）与交换（may_swap=0），除非后续降了priority。
		 * - 若本轮没有回收到页，将会中止boost回收，避免无效循环。
		 */
		sc.may_writepage = !laptop_mode && !nr_boost_reclaim;
		sc.may_swap = !nr_boost_reclaim;

		/*
		 * Do some background aging, to give pages a chance to be
		 * referenced before reclaiming. All pages are rotated
		 * regardless of classzone as this is about consistent aging.
		 */
		/*
		 * kswapd老化，分MGLRU页面老化和传统LRU页面老化
		 *	- MGLRU老化，老化匿名页面和文件页面
		 *	- 传统LRU老化，只老化匿名页面
		 */
		kswapd_age_node(pgdat, &sc);

		/*
		 * If we're getting trouble reclaiming, start doing writepage
		 * even in laptop mode.
		 */
		/*
		 * sc.priority < DEF_PRIORITY - 2，
		 * 表示至少已经进行了三轮kswap回收，还没回收到足够的内存
		 * 则开始使能writebakc, 不管是laptop模式还是boost回收。
		 */
		if (sc.priority < DEF_PRIORITY - 2)
			sc.may_writepage = 1;

		/* Call soft limit reclaim before calling shrink_node. */
		/* 阐释软限额回收，memcg v1，先不看，cgroup v2不会用这个 */
		sc.nr_scanned = 0;
		nr_soft_scanned = 0;
		nr_soft_reclaimed = memcg1_soft_limit_reclaim(pgdat, sc.order,
							      sc.gfp_mask, &nr_soft_scanned);
		sc.nr_reclaimed += nr_soft_reclaimed;

		/*
		 * There should be no need to raise the scanning priority if
		 * enough pages are already being scanned that that high
		 * watermark would be met at 100% efficiency.
		 */
		if (kswapd_shrink_node(pgdat, &sc))
			raise_priority = false;

		/*
		 * If the low watermark is met there is no need for processes
		 * to be throttled on pfmemalloc_wait as they should not be
		 * able to safely make forward progress. Wake them
		 */
		/*
		 * 如果直接回收流程被节流了, throttle_direct_reclaim()
		 * 并且目前可以允许直接回收（kswapd无法回收内存了，或者当前空闲内存可以满足pfmemalloc需求）
		 * 则唤醒被节流的直接回收流程
		 */
		if (waitqueue_active(&pgdat->pfmemalloc_wait) &&
				allow_direct_reclaim(pgdat))
			wake_up_all(&pgdat->pfmemalloc_wait);

		/* Check if kswapd should be suspending */
		__fs_reclaim_release(_THIS_IP_);
		ret = kthread_freezable_should_stop(&was_frozen);
		__fs_reclaim_acquire(_THIS_IP_);
		if (was_frozen || ret)
			break;

		/*
		 * Raise priority if scanning rate is too low or there was no
		 * progress in reclaiming pages
		 */
		nr_reclaimed = sc.nr_reclaimed - nr_reclaimed;
		nr_boost_reclaim -= min(nr_boost_reclaim, nr_reclaimed);

		/*
		 * If reclaim made no progress for a boost, stop reclaim as
		 * IO cannot be queued and it could be an infinite loop in
		 * extreme circumstances.
		 */
		if (nr_boost_reclaim && !nr_reclaimed)
			break;

		if (raise_priority || !nr_reclaimed)
			sc.priority--;
	} while (sc.priority >= 1);

	/*
	 * Restart only if it went through the priority loop all the way,
	 * but cache_trim_mode didn't work.
	 */
	if (!sc.nr_reclaimed && sc.priority < 1 &&
	    !sc.no_cache_trim_mode && sc.cache_trim_mode_failed) {
		sc.no_cache_trim_mode = 1;
		goto restart;
	}

	if (!sc.nr_reclaimed)
		pgdat->kswapd_failures++;

out:
	clear_reclaim_active(pgdat, highest_zoneidx);

	/* If reclaim was boosted, account for the reclaim done in this pass */
	if (boosted) {
		unsigned long flags;

		for (i = 0; i <= highest_zoneidx; i++) {
			if (!zone_boosts[i])
				continue;

			/* Increments are under the zone lock */
			zone = pgdat->node_zones + i;
			spin_lock_irqsave(&zone->lock, flags);
			/* 恢复zone的watermark_boost */
			zone->watermark_boost -= min(zone->watermark_boost, zone_boosts[i]);
			spin_unlock_irqrestore(&zone->lock, flags);
		}

		/*
		 * As there is now likely space, wakeup kcompact to defragment
		 * pageblocks.
		 */
		wakeup_kcompactd(pgdat, pageblock_order, highest_zoneidx);
	}

	snapshot_refaults(NULL, pgdat);
	__fs_reclaim_release(_THIS_IP_);
	psi_memstall_leave(&pflags);
	set_task_reclaim_state(current, NULL);

	/*
	 * Return the order kswapd stopped reclaiming at as
	 * prepare_kswapd_sleep() takes it into account. If another caller
	 * entered the allocator slow path while kswapd was awake, order will
	 * remain at the higher level.
	 */
	return sc.order;
}

/*
 * The pgdat->kswapd_highest_zoneidx is used to pass the highest zone index to
 * be reclaimed by kswapd from the waker. If the value is MAX_NR_ZONES which is
 * not a valid index then either kswapd runs for first time or kswapd couldn't
 * sleep after previous reclaim attempt (node is still unbalanced). In that
 * case return the zone index of the previous kswapd reclaim cycle.
 */
static enum zone_type kswapd_highest_zoneidx(pg_data_t *pgdat,
					   enum zone_type prev_highest_zoneidx)
{
	enum zone_type curr_idx = READ_ONCE(pgdat->kswapd_highest_zoneidx);

	return curr_idx == MAX_NR_ZONES ? prev_highest_zoneidx : curr_idx;
}

/*
 * 等待队列使用流程
 *
 * 1.创建等待队列头
 *	DECLARE_WAIT_QUEUE_HEAD(my_wq);
 * 2.创建等待队列项
 *	DEFINE_WAIT(my_wait);
 * 3.将等待队列项加入等待队列
 *	prepare_to_wait(&my_wq, &my_wait, TASK_INTERRUPTIBLE)
 * 4.调度出去
 *	schedule/schedule_timeout
 * 5.唤醒阶段（其它线程或者中断）
 *	wake_up(&my_wq);/wake_up_all(&my_wa);
 * 6.唤醒后，清理阶段，从等待队列移除
 *	finish_wait(&my_wq, &my_wait);
 */

/*
 * kswapd尝试进入休眠
 *
 * 1.加入等待队列；
 * 2.prepare_kswapd_sleep()判断能否进入休眠，可以进入休眠的条件：
 *	- kswapd无法在当前节点回收到内存;
 *	- 当前node节点达到平衡条件，空闲内存满足水位线
 * 3.如果可以进入休眠，先唤醒kcompactd, 再进入短暂休眠，100ms;
 * 4.如果短暂休眠完整做完，再次判断能否进入西休眠，如果可以，则进入完全休眠直到被唤醒;
 */
static void kswapd_try_to_sleep(pg_data_t *pgdat, int alloc_order, int reclaim_order,
				unsigned int highest_zoneidx)
{
	long remaining = 0;
	DEFINE_WAIT(wait);

	if (freezing(current) || kthread_should_stop())
		return;

	/* 加入等待队列 */
	prepare_to_wait(&pgdat->kswapd_wait, &wait, TASK_INTERRUPTIBLE);

	/*
	 * Try to sleep for a short interval. Note that kcompactd will only be
	 * woken if it is possible to sleep for a short interval. This is
	 * deliberate on the assumption that if reclaim cannot keep an
	 * eligible zone balanced that it's also unlikely that compaction will
	 * succeed.
	 */
	/*
	 * kswapd可以休眠的条件：
	 *	1.kswapd无法在当前节点回收到内存;
	 *	2.当前node节点达到平衡条件，空闲内存满足水位线
	 */
	if (prepare_kswapd_sleep(pgdat, reclaim_order, highest_zoneidx)) {
		/*
		 * Compaction records what page blocks it recently failed to
		 * isolate pages from and skips them in the future scanning.
		 * When kswapd is going to sleep, it is reasonable to assume
		 * that pages and compaction may succeed so reset the cache.
		 */
		reset_isolation_suitable(pgdat);

		/*
		 * We have freed the memory, now we should compact it to make
		 * allocation of the requested order possible.
		 */
		/* 上面判断kswapd可以进入短时间休眠，说明已经释放了一些内存，可以尝试页面规整 */
		wakeup_kcompactd(pgdat, alloc_order, highest_zoneidx);

		/* 休眠100ms */
		remaining = schedule_timeout(HZ/10);

		/*
		 * If woken prematurely then reset kswapd_highest_zoneidx and
		 * order. The values will either be from a wakeup request or
		 * the previous request that slept prematurely.
		 */
		/*
		 * 如果提前被唤醒，则重置kswapd_highest_zoneidx和order
		 * 这些值要么来自唤醒请求，要么来自之前提前睡眠的请求
		 */
		if (remaining) {
			WRITE_ONCE(pgdat->kswapd_highest_zoneidx,
					kswapd_highest_zoneidx(pgdat,
							highest_zoneidx));

			if (READ_ONCE(pgdat->kswapd_order) < reclaim_order)
				WRITE_ONCE(pgdat->kswapd_order, reclaim_order);
		}

		finish_wait(&pgdat->kswapd_wait, &wait);
		prepare_to_wait(&pgdat->kswapd_wait, &wait, TASK_INTERRUPTIBLE);
	}

	/*
	 * After a short sleep, check if it was a premature sleep. If not, then
	 * go fully to sleep until explicitly woken up.
	 */
	/*
	 * 经历过短暂的休眠, 再次判断能否进入休眠，如果可以，则进入完全休眠直到唤醒
	 */
	if (!remaining &&
	    prepare_kswapd_sleep(pgdat, reclaim_order, highest_zoneidx)) {
		trace_mm_vmscan_kswapd_sleep(pgdat->node_id);

		/*
		 * vmstat counters are not perfectly accurate and the estimated
		 * value for counters such as NR_FREE_PAGES can deviate from the
		 * true value by nr_online_cpus * threshold. To avoid the zone
		 * watermarks being breached while under pressure, we reduce the
		 * per-cpu vmstat threshold while kswapd is awake and restore
		 * them before going back to sleep.
		 */
		/*
		 * vmstat计数器不完全准确，诸如NR_FREE_PAGES之类的估计值
		 * 可能与真实值相差nr_online_cpus * threshold。
		 * 为了避免在压力下突破zone水位线，我们在kswapd唤醒时
		 * 降低per-CPU vmstat阈值，并在返回睡眠前恢复它们
		 */
		set_pgdat_percpu_threshold(pgdat, calculate_normal_threshold);

		/* 完全睡眠直到被唤醒 */
		if (!kthread_should_stop())
			schedule();

		set_pgdat_percpu_threshold(pgdat, calculate_pressure_threshold);
	} else {
		if (remaining)
			count_vm_event(KSWAPD_LOW_WMARK_HIT_QUICKLY);
		else
			count_vm_event(KSWAPD_HIGH_WMARK_HIT_QUICKLY);
	}
	finish_wait(&pgdat->kswapd_wait, &wait);
}

/*
 * The background pageout daemon, started as a kernel thread
 * from the init process.
 *
 * This basically trickles out pages so that we have _some_
 * free memory available even if there is no other activity
 * that frees anything up. This is needed for things like routing
 * etc, where we otherwise might have all activity going on in
 * asynchronous contexts that cannot page things out.
 *
 * If there are applications that are active memory-allocators
 * (most normal use), this basically shouldn't matter.
 */
static int kswapd(void *p)
{
	unsigned int alloc_order, reclaim_order;
	unsigned int highest_zoneidx = MAX_NR_ZONES - 1;
	pg_data_t *pgdat = (pg_data_t *)p;	/* 当前内存node */
	struct task_struct *tsk = current;
	const struct cpumask *cpumask = cpumask_of_node(pgdat->node_id);

	if (!cpumask_empty(cpumask))
		set_cpus_allowed_ptr(tsk, cpumask);

	/*
	 * Tell the memory management that we're a "memory allocator",
	 * and that if we need more memory we should get access to it
	 * regardless (see "__alloc_pages()"). "kswapd" should
	 * never get caught in the normal page freeing logic.
	 *
	 * (Kswapd normally doesn't need memory anyway, but sometimes
	 * you need a small amount of memory in order to be able to
	 * page out something else, and this flag essentially protects
	 * us from recursively trying to free more memory as we're
	 * trying to free the first piece of memory in the first place).
	 */
	tsk->flags |= PF_MEMALLOC | PF_KSWAPD;
	set_freezable();

	WRITE_ONCE(pgdat->kswapd_order, 0);
	WRITE_ONCE(pgdat->kswapd_highest_zoneidx, MAX_NR_ZONES);
	atomic_set(&pgdat->nr_writeback_throttled, 0);
	for ( ; ; ) {
		bool was_frozen;

		alloc_order = reclaim_order = READ_ONCE(pgdat->kswapd_order);
		highest_zoneidx = kswapd_highest_zoneidx(pgdat,
							highest_zoneidx);

kswapd_try_sleep:
		/*
		 * 尝试让kswapd进入休眠
		 * 1.根据free内存和水位线判断能否进入休眠；
		 * 2.先唤醒kcompactd，再进行短暂休眠，100ms；
		 * 3.短暂休眠没被打断，并且还可以进入休眠，则进入完全休眠直到被唤醒;
		 */
		kswapd_try_to_sleep(pgdat, alloc_order, reclaim_order,
					highest_zoneidx);

		/* Read the new order and highest_zoneidx */
		/* 被唤醒后，读取新的回收参数 */
		alloc_order = READ_ONCE(pgdat->kswapd_order);
		highest_zoneidx = kswapd_highest_zoneidx(pgdat,
							highest_zoneidx);
		WRITE_ONCE(pgdat->kswapd_order, 0);
		WRITE_ONCE(pgdat->kswapd_highest_zoneidx, MAX_NR_ZONES);

		/* 判断是否需要停止kaswap线程， 如果需要则跳出循环退出 */
		if (kthread_freezable_should_stop(&was_frozen))
			break;

		/*
		 * We can speed up thawing tasks if we don't call balance_pgdat
		 * after returning from the refrigerator
		 */
		/* 如果是从冻结中唤醒，则不往下执行balance_pgdat, 可以加速解冻任务 */
		if (was_frozen)
			continue;

		/*
		 * Reclaim begins at the requested order but if a high-order
		 * reclaim fails then kswapd falls back to reclaiming for
		 * order-0. If that happens, kswapd will consider sleeping
		 * for the order it finished reclaiming at (reclaim_order)
		 * but kcompactd is woken to compact for the original
		 * request (alloc_order).
		 */
		/*
		 * 回收从请求的order开始，但如果高阶回收失败，
		 * kswapd会回退到order-0回收。如果发生这种情况，
		 * kswapd将考虑在完成回收的order（reclaim_order）处睡眠，
		 * 但会唤醒kcompactd来压缩原始请求（alloc_order）
		 */
		trace_mm_vmscan_kswapd_wake(pgdat->node_id, highest_zoneidx,
						alloc_order);
		/* kswapd回收内存核心操作 */
		reclaim_order = balance_pgdat(pgdat, alloc_order,
						highest_zoneidx);
		/* 如果回收到的order小于请求order，则重新尝试休眠（会唤醒kcompactd）*/
		if (reclaim_order < alloc_order)
			goto kswapd_try_sleep;
	}

	tsk->flags &= ~(PF_MEMALLOC | PF_KSWAPD);

	return 0;
}

/*
 * A zone is low on free memory or too fragmented for high-order memory.  If
 * kswapd should reclaim (direct reclaim is deferred), wake it up for the zone's
 * pgdat.  It will wake up kcompactd after reclaiming memory.  If kswapd reclaim
 * has failed or is not needed, still wake up kcompactd if only compaction is
 * needed.
 */
void wakeup_kswapd(struct zone *zone, gfp_t gfp_flags, int order,
		   enum zone_type highest_zoneidx)
{
	pg_data_t *pgdat;
	enum zone_type curr_idx;

	if (!managed_zone(zone))
		return;

	if (!cpuset_zone_allowed(zone, gfp_flags))
		return;

	pgdat = zone->zone_pgdat;
	curr_idx = READ_ONCE(pgdat->kswapd_highest_zoneidx);

	if (curr_idx == MAX_NR_ZONES || curr_idx < highest_zoneidx)
		WRITE_ONCE(pgdat->kswapd_highest_zoneidx, highest_zoneidx);

	if (READ_ONCE(pgdat->kswapd_order) < order)
		WRITE_ONCE(pgdat->kswapd_order, order);

	if (!waitqueue_active(&pgdat->kswapd_wait))
		return;

	/* Hopeless node, leave it to direct reclaim if possible */
	if (pgdat->kswapd_failures >= MAX_RECLAIM_RETRIES ||
	    (pgdat_balanced(pgdat, order, highest_zoneidx) &&
	     !pgdat_watermark_boosted(pgdat, highest_zoneidx))) {
		/*
		 * There may be plenty of free memory available, but it's too
		 * fragmented for high-order allocations.  Wake up kcompactd
		 * and rely on compaction_suitable() to determine if it's
		 * needed.  If it fails, it will defer subsequent attempts to
		 * ratelimit its work.
		 */
		if (!(gfp_flags & __GFP_DIRECT_RECLAIM))
			wakeup_kcompactd(pgdat, order, highest_zoneidx);
		return;
	}

	trace_mm_vmscan_wakeup_kswapd(pgdat->node_id, highest_zoneidx, order,
				      gfp_flags);
	wake_up_interruptible(&pgdat->kswapd_wait);
}

#ifdef CONFIG_HIBERNATION
/*
 * Try to free `nr_to_reclaim' of memory, system-wide, and return the number of
 * freed pages.
 *
 * Rather than trying to age LRUs the aim is to preserve the overall
 * LRU order by reclaiming preferentially
 * inactive > active > active referenced > active mapped
 */
unsigned long shrink_all_memory(unsigned long nr_to_reclaim)
{
	struct scan_control sc = {
		.nr_to_reclaim = nr_to_reclaim,
		.gfp_mask = GFP_HIGHUSER_MOVABLE,
		.reclaim_idx = MAX_NR_ZONES - 1,
		.priority = DEF_PRIORITY,
		.may_writepage = 1,
		.may_unmap = 1,
		.may_swap = 1,
		.hibernation_mode = 1,
	};
	struct zonelist *zonelist = node_zonelist(numa_node_id(), sc.gfp_mask);
	unsigned long nr_reclaimed;
	unsigned int noreclaim_flag;

	fs_reclaim_acquire(sc.gfp_mask);
	noreclaim_flag = memalloc_noreclaim_save();
	set_task_reclaim_state(current, &sc.reclaim_state);

	nr_reclaimed = do_try_to_free_pages(zonelist, &sc);

	set_task_reclaim_state(current, NULL);
	memalloc_noreclaim_restore(noreclaim_flag);
	fs_reclaim_release(sc.gfp_mask);

	return nr_reclaimed;
}
#endif /* CONFIG_HIBERNATION */

/*
 * This kswapd start function will be called by init and node-hot-add.
 */
void __meminit kswapd_run(int nid)
{
	/* 根据mem node id获取mem node结构体 */
	pg_data_t *pgdat = NODE_DATA(nid);

	pgdat_kswapd_lock(pgdat);
	if (!pgdat->kswapd) {
		pgdat->kswapd = kthread_run(kswapd, pgdat, "kswapd%d", nid);
		if (IS_ERR(pgdat->kswapd)) {
			/* failure at boot is fatal */
			pr_err("Failed to start kswapd on node %d，ret=%ld\n",
				   nid, PTR_ERR(pgdat->kswapd));
			BUG_ON(system_state < SYSTEM_RUNNING);
			pgdat->kswapd = NULL;
		}
	}
	pgdat_kswapd_unlock(pgdat);
}

/*
 * Called by memory hotplug when all memory in a node is offlined.  Caller must
 * be holding mem_hotplug_begin/done().
 */
void __meminit kswapd_stop(int nid)
{
	pg_data_t *pgdat = NODE_DATA(nid);
	struct task_struct *kswapd;

	pgdat_kswapd_lock(pgdat);
	kswapd = pgdat->kswapd;
	if (kswapd) {
		kthread_stop(kswapd);
		pgdat->kswapd = NULL;
	}
	pgdat_kswapd_unlock(pgdat);
}

static int __init kswapd_init(void)
{
	int nid;

	swap_setup();
	/* per-mem_node kswapd thread */
	/*
	 * 每个memory node有一个kswapd_run
	 * kswapd是per-mem_node的
	 */
	for_each_node_state(nid, N_MEMORY)
 		kswapd_run(nid);
	return 0;
}

module_init(kswapd_init)

#ifdef CONFIG_NUMA
/*
 * Node reclaim mode
 *
 * If non-zero call node_reclaim when the number of free pages falls below
 * the watermarks.
 */
int node_reclaim_mode __read_mostly;

/*
 * Priority for NODE_RECLAIM. This determines the fraction of pages
 * of a node considered for each zone_reclaim. 4 scans 1/16th of
 * a zone.
 */
#define NODE_RECLAIM_PRIORITY 4

/*
 * Percentage of pages in a zone that must be unmapped for node_reclaim to
 * occur.
 */
int sysctl_min_unmapped_ratio = 1;

/*
 * If the number of slab pages in a zone grows beyond this percentage then
 * slab reclaim needs to occur.
 */
int sysctl_min_slab_ratio = 5;

static inline unsigned long node_unmapped_file_pages(struct pglist_data *pgdat)
{
	unsigned long file_mapped = node_page_state(pgdat, NR_FILE_MAPPED);
	unsigned long file_lru = node_page_state(pgdat, NR_INACTIVE_FILE) +
		node_page_state(pgdat, NR_ACTIVE_FILE);

	/*
	 * It's possible for there to be more file mapped pages than
	 * accounted for by the pages on the file LRU lists because
	 * tmpfs pages accounted for as ANON can also be FILE_MAPPED
	 */
	return (file_lru > file_mapped) ? (file_lru - file_mapped) : 0;
}

/* Work out how many page cache pages we can reclaim in this reclaim_mode */
static unsigned long node_pagecache_reclaimable(struct pglist_data *pgdat)
{
	unsigned long nr_pagecache_reclaimable;
	unsigned long delta = 0;

	/*
	 * If RECLAIM_UNMAP is set, then all file pages are considered
	 * potentially reclaimable. Otherwise, we have to worry about
	 * pages like swapcache and node_unmapped_file_pages() provides
	 * a better estimate
	 */
	if (node_reclaim_mode & RECLAIM_UNMAP)
		nr_pagecache_reclaimable = node_page_state(pgdat, NR_FILE_PAGES);
	else
		nr_pagecache_reclaimable = node_unmapped_file_pages(pgdat);

	/* If we can't clean pages, remove dirty pages from consideration */
	if (!(node_reclaim_mode & RECLAIM_WRITE))
		delta += node_page_state(pgdat, NR_FILE_DIRTY);

	/* Watch for any possible underflows due to delta */
	if (unlikely(delta > nr_pagecache_reclaimable))
		delta = nr_pagecache_reclaimable;

	return nr_pagecache_reclaimable - delta;
}

/*
 * Try to free up some pages from this node through reclaim.
 */
static int __node_reclaim(struct pglist_data *pgdat, gfp_t gfp_mask, unsigned int order)
{
	/* Minimum pages needed in order to stay on node */
	const unsigned long nr_pages = 1 << order;
	struct task_struct *p = current;
	unsigned int noreclaim_flag;
	struct scan_control sc = {
		.nr_to_reclaim = max(nr_pages, SWAP_CLUSTER_MAX),
		.gfp_mask = current_gfp_context(gfp_mask),
		.order = order,
		.priority = NODE_RECLAIM_PRIORITY,
		.may_writepage = !!(node_reclaim_mode & RECLAIM_WRITE),
		.may_unmap = !!(node_reclaim_mode & RECLAIM_UNMAP),
		.may_swap = 1,
		.reclaim_idx = gfp_zone(gfp_mask),
	};
	unsigned long pflags;

	trace_mm_vmscan_node_reclaim_begin(pgdat->node_id, order,
					   sc.gfp_mask);

	cond_resched();
	psi_memstall_enter(&pflags);
	delayacct_freepages_start();
	fs_reclaim_acquire(sc.gfp_mask);
	/*
	 * We need to be able to allocate from the reserves for RECLAIM_UNMAP
	 */
	noreclaim_flag = memalloc_noreclaim_save();
	set_task_reclaim_state(p, &sc.reclaim_state);

	if (node_pagecache_reclaimable(pgdat) > pgdat->min_unmapped_pages ||
	    node_page_state_pages(pgdat, NR_SLAB_RECLAIMABLE_B) > pgdat->min_slab_pages) {
		/*
		 * Free memory by calling shrink node with increasing
		 * priorities until we have enough memory freed.
		 */
		do {
			shrink_node(pgdat, &sc);
		} while (sc.nr_reclaimed < nr_pages && --sc.priority >= 0);
	}

	set_task_reclaim_state(p, NULL);
	memalloc_noreclaim_restore(noreclaim_flag);
	fs_reclaim_release(sc.gfp_mask);
	psi_memstall_leave(&pflags);
	delayacct_freepages_end();

	trace_mm_vmscan_node_reclaim_end(sc.nr_reclaimed);

	return sc.nr_reclaimed >= nr_pages;
}

int node_reclaim(struct pglist_data *pgdat, gfp_t gfp_mask, unsigned int order)
{
	int ret;

	/*
	 * Node reclaim reclaims unmapped file backed pages and
	 * slab pages if we are over the defined limits.
	 *
	 * A small portion of unmapped file backed pages is needed for
	 * file I/O otherwise pages read by file I/O will be immediately
	 * thrown out if the node is overallocated. So we do not reclaim
	 * if less than a specified percentage of the node is used by
	 * unmapped file backed pages.
	 */
	if (node_pagecache_reclaimable(pgdat) <= pgdat->min_unmapped_pages &&
	    node_page_state_pages(pgdat, NR_SLAB_RECLAIMABLE_B) <=
	    pgdat->min_slab_pages)
		return NODE_RECLAIM_FULL;

	/*
	 * Do not scan if the allocation should not be delayed.
	 */
	if (!gfpflags_allow_blocking(gfp_mask) || (current->flags & PF_MEMALLOC))
		return NODE_RECLAIM_NOSCAN;

	/*
	 * Only run node reclaim on the local node or on nodes that do not
	 * have associated processors. This will favor the local processor
	 * over remote processors and spread off node memory allocations
	 * as wide as possible.
	 */
	if (node_state(pgdat->node_id, N_CPU) && pgdat->node_id != numa_node_id())
		return NODE_RECLAIM_NOSCAN;

	if (test_and_set_bit(PGDAT_RECLAIM_LOCKED, &pgdat->flags))
		return NODE_RECLAIM_NOSCAN;

	ret = __node_reclaim(pgdat, gfp_mask, order);
	clear_bit(PGDAT_RECLAIM_LOCKED, &pgdat->flags);

	if (ret)
		count_vm_event(PGSCAN_ZONE_RECLAIM_SUCCESS);
	else
		count_vm_event(PGSCAN_ZONE_RECLAIM_FAILED);

	return ret;
}
#endif

/**
 * check_move_unevictable_folios - Move evictable folios to appropriate zone
 * lru list
 * @fbatch: Batch of lru folios to check.
 *
 * Checks folios for evictability, if an evictable folio is in the unevictable
 * lru list, moves it to the appropriate evictable lru list. This function
 * should be only used for lru folios.
 */
void check_move_unevictable_folios(struct folio_batch *fbatch)
{
	struct lruvec *lruvec = NULL;
	int pgscanned = 0;
	int pgrescued = 0;
	int i;

	for (i = 0; i < fbatch->nr; i++) {
		struct folio *folio = fbatch->folios[i];
		int nr_pages = folio_nr_pages(folio);

		pgscanned += nr_pages;

		/* block memcg migration while the folio moves between lrus */
		if (!folio_test_clear_lru(folio))
			continue;

		lruvec = folio_lruvec_relock_irq(folio, lruvec);
		if (folio_evictable(folio) && folio_test_unevictable(folio)) {
			lruvec_del_folio(lruvec, folio);
			folio_clear_unevictable(folio);
			lruvec_add_folio(lruvec, folio);
			pgrescued += nr_pages;
		}
		folio_set_lru(folio);
	}

	if (lruvec) {
		__count_vm_events(UNEVICTABLE_PGRESCUED, pgrescued);
		__count_vm_events(UNEVICTABLE_PGSCANNED, pgscanned);
		unlock_page_lruvec_irq(lruvec);
	} else if (pgscanned) {
		count_vm_events(UNEVICTABLE_PGSCANNED, pgscanned);
	}
}
EXPORT_SYMBOL_GPL(check_move_unevictable_folios);

/*
 * 传统LRU回收
  kswapd
	--> balance_pgdat
		--> kswapd_age_node
		--> memcg1_soft_limit_reclaim
		--> kswapd_shrink_node
			--> shrink_node
				--> prepare_scan_control
				--> shrink_node_memcgs
					--> shrink_lruvec
						--> lru_gen_shrink_lruvec
						--> get_scan_count [only called by shrink_list()]
						--> shrink_list
							--> shrink_inactive_list
								--> isolate_lru_folios
								--> shrink_folio_list
									--> folio_check_references
									--> try_to_unmap
									--> pageout
									--> filemap_release_folio
									--> __remove_mapping
									--> free_unref_folios
								--> move_folios_to_lru
							--> shrink_active_list
						--> shrink_active_list
					--> shrink_slab


  kswapd_run()                                    [mm/vmscan.c:9161]
    └─ kthread_run(kswapd, pgdat, ...)
         └─ kswapd()                              [mm/vmscan.c:9174]
              │  tsk->flags |= PF_MEMALLOC | PF_KSWAPD
              │  for (;;) {
              ├─ kswapd_try_to_sleep()            [mm/vmscan.c:9055]
              │    ├─ prepare_to_wait()           ← 设置 TASK_INTERRUPTIBLE
              │    ├─ wake_up_process(kcompactd)  ← 唤醒内存规整线程
              │    └─ schedule()                  ← 休眠，等待 wakeup_kswapd() 唤醒
              │
              ├─ kswapd_age_node
              │    │
              │    │ // 传统LRU老化路径
              │    │ // 如果匿名页不可以老化(没有swap\支持demotion)，或者inactive匿名页还比较多，则不老化
              │    │ if (!can_age_anon_pages(pgdat, sc))
              │    │ if (!inactive_is_low(lruvec, LRU_INACTIVE_ANON))
              │    │	return;
              │    │
              │    └─ shrink_active_list(LRU_ACTIVE_ANON)	//传统LRU，从root memcg向下遍历, 这里只老化anon list
              │
              └─ balance_pgdat(pgdat, order, highest_zoneidx)  [mm/vmscan.c:8732]
                   │  sc.priority = DEF_PRIORITY  (初始优先级=12)
                   │  do {                        ← 优先级递减循环
                   │
                   ├─ [可选] memcg1_soft_limit_reclaim()   ← v1 软限制回收
                   │
                   ├─ kswapd_shrink_node(pgdat, &sc)        [mm/vmscan.c:8615]
                   │    │  sc->nr_to_reclaim = Σ max(high_wmark, SWAP_CLUSTER_MAX)
                   │    └─ shrink_node(pgdat, sc)            [mm/vmscan.c:7476]
                   │         │
                   │         │  ← 传统LRU路径（MGLRU未开启 或 非全局回收）
                   │         ├─ prepare_scan_control()       ← 更新 anon_cost/file_cost 等
                   │         └─ shrink_node_memcgs(pgdat, sc) [mm/vmscan.c:7381]
                   │              │  partial=NULL（kswapd 总是完整遍历）
                   │              │  for each memcg in tree:
                   │              ├─ mem_cgroup_calculate_protection()
                   │              ├─ [skip] below_min → continue（硬保护）
                   │              ├─ [skip] below_low && !low_reclaim → continue（软保护）
                   │              ├─ shrink_lruvec(lruvec, sc)  [mm/vmscan.c:7137]
                   │              │    │
                   │              │    ├─ get_scan_count(lruvec, sc, nr[])
                   │              │    │    └─ 按 anon/file 比例分配各 LRU 链表扫描数
                   │              │    │       综合考虑的因素包括：swappiness\sc->priority\memcg的内存限制
                   │              │    │
                   │              │    │  blk_start_plug()      ← 合并块设备 I/O
                   │              │    │  while (nr[INACTIVE_ANON] || nr[ACTIVE_FILE] || nr[INACTIVE_FILE]):
                   │              │    │    for_each_evictable_lru(lru):  ← 遍历 4 条链表
                   │              │    │      nr_to_scan = min(nr[lru], SWAP_CLUSTER_MAX)
                   │              │    │
                   │              │    ├─ shrink_list(lru, nr_to_scan, lruvec, sc)  [mm/vmscan.c:3030]
                   │              │    │    ├─ is_active_lru(lru)?
                   │              │    │    │    └─ shrink_active_list()        // 可能会老化file或者anon list
                   │              │    │    │         ├─ isolate_lru_folios()       ← 从 active 尾部隔离
                   │              │    │    │         ├─ folio_check_references()   ← 判断引用热度
                   │              │    │    │         ├─ → l_active: 重新放回 active（最近访问）
                   │              │    │    │         └─ → l_inactive: 降级到 inactive 链表头
                   │              │    │    │
                   │              │    │    └─ !active:
                   │              │    │         └─ shrink_inactive_list()         [mm/vmscan.c:2637]
                   │              │    │              ├─ too_many_isolated()? → reclaim_throttle()
                   │              │    │              ├─ lru_add_drain()
                   │              │    │              ├─ isolate_lru_folios()  ← 从 inactive 尾部隔离
                   │              │    │              ├─ shrink_folio_list(&folio_list, ...)  [mm/vmscan.c:1446]
                   │              │    │              │    │  逐个处理隔离出的 folio：
                   │              │    │              │    ├─ folio_trylock()
                   │              │    │              │    ├─ folio_check_references()        ← 引用检测
                   │              │    │              │    │    ├─ FOLIOREF_ACTIVATE  → 放回 active
                   │              │    │              │    │    ├─ FOLIOREF_KEEP      → 放回 inactive
                   │              │    │              │    │    └─ FOLIOREF_RECLAIM   → 继续回收
                   │              │    │              │    ├─ [匿名页] add_to_swap()  ← 分配 swap 空间
                   │              │    │              │    ├─ [mapped] try_to_unmap() ← 解除所有页表映射
                   │              │    │              │    ├─ [dirty] pageout()    ← 触发回写
                   │              │    │              │    │    └─ mapping->a_ops->writepage()
                   │              │    │              │    ├─ [clean] __remove_mapping() ← 从 page cache 删除
                   │              │    │              │    └─ free_unref_page() / 放入 free_pages list
                   │              │    │              └─ move_folio_to_lru()   ← 未回收的放回LRU链表
                   │              │    │
                   │              │    ├─ blk_finish_plug()
                   │              │    │
                   │              │    └─ shrink_active_list()
                   │              │
                   │              └─ shrink_slab(gfp, nid, memcg, priority)  ← slab/dentry/inode 回收
                   │
                   │  } while (--sc->priority >= 0 && !pgdat_balanced())
                   │
                   └─ [balanced] 退出，kswapd 回到 kswapd_try_to_sleep()

  kswapd 唤醒触发点：
  __alloc_pages()
    └─ get_page_from_freelist()  ← 快速路径失败（低于 low watermark）
         └─ wake_all_kswapds()   [mm/page_alloc.c:3995]
              └─ wakeup_kswapd(zone, order, highest_zoneidx)
                   └─ wake_up_interruptible(&pgdat->kswapd_wait)

  ---
  二、直接回收（Direct Reclaim）路径

  __alloc_pages(gfp_mask, order, ...)              [mm/page_alloc.c:4770]
    ├─ get_page_from_freelist()                    ← 快速路径（低于 min watermark → 失败）
    └─ __alloc_pages_slowpath(gfp_mask, order, ac) [mm/page_alloc.c:4226]
         │
         ├─ wake_all_kswapds()                     ← 顺便唤醒 kswapd
         ├─ get_page_from_freelist()               ← 放宽水位线再试一次
         │
         │  [仍然失败，尝试直接回收]
         │  if (!can_direct_reclaim) → goto nopage
         │  if (current->flags & PF_MEMALLOC) → goto nopage  ← 防递归
         │
         ├─ __alloc_pages_direct_compact(...)	// 先尝试compaction
         │
         │  [compaction仍然失败，开始直接回收]
         │
         ├─ __alloc_pages_direct_reclaim(...)	// 直接回收
         │    │
         │    ├─ __perform_reclaim(gfp_mask, order, ac)  [mm/page_alloc.c:3936]
         │    │    │  fs_reclaim_acquire()          ← 防止文件系统递归
         │    │    │  memalloc_noreclaim_save()
         │    │    │
         │    │    └─ try_to_free_pages(zonelist, order, gfp_mask, nodemask)
         │    │         │                          [mm/vmscan.c:8244]
         │    │         │  sc = { .nr_to_reclaim = SWAP_CLUSTER_MAX,
         │    │         │         .priority = DEF_PRIORITY, .may_swap = 1, ... }
         │    │         │
         │    │         ├─ throttle_direct_reclaim()  ← 进入 pfmemalloc_wait 等 kswapd 唤醒
	 │    │         │    └─ 若 kswapd 在追赶则节流，减少直接回收竞争
         │    │         │
         │    │         └─ do_try_to_free_pages(zonelist, &sc)  [mm/vmscan.c:7897]
         │    │              │
         │    │              │  do {               ← 优先级递减循环（12 → 0）
         │    │              │    if (priority < DEF_PRIORITY-2) sc->may_writepage = 1
         │    │              │
         │    │              └─ shrink_zones(zonelist, sc)       [mm/vmscan.c:7728]
         │    │                   │  for_each_zone_zonelist():
         │    │                   │    [skip] !cpuset_zone_allowed → continue
         │    │                   │    [skip] zone already balanced → continue
         │    │                   │    [可选] memcg1_soft_limit_reclaim()
         │    │                   │
         │    │                   └─ shrink_node(pgdat, sc)       [mm/vmscan.c:7476]
         │    │                        └─ shrink_node_memcgs()
         │    │                             └─ shrink_lruvec()
         │    │                                  └─ ... [同 kswapd 路径，见上]
         │    └─ get_page_from_freelist()
         │
         ├─ __alloc_pages_direct_compact(...)	// 再次尝试compaction
         │
         ├─ __alloc_pages_may_oom(...)
	 │    │
	 │    ├─ get_page_from_freelist()	//再尝试获取内存，如果还是失败，则走oom
	 │    └─ out_of_memory()
	 │        │
	 │        ├─ select_bad_process		//遍历所有task，选出分数最高的task
	 │        │   └─ oom_evaluate_task	//遍历所有task
	 │        │       └─  oom_badness	//内存使用最多的(rss + swapents + pagetables)，得分最高；设置oom_score_adj为-1000ke可以避免被杀
	 │        └─ oom_kill_process		//杀掉选择的task
	 │
	 └─ 如果oom杀掉了进程，则重新尝试分配内存，跳转到前面

 * MGLRU回收流程
 * kswapd
 *
  kswapd()                          [vmscan.c: ~8800]
    └─ balance_pgdat()              [vmscan.c: ~8358]
         ├─ kswapd_age_node()       [vmscan.c: ~8008]  ← MGLRU 老化入口
         │    └─ lru_gen_age_node() [vmscan.c: ~5065]
         └─ kswapd_shrink_node()    [vmscan.c: ~8241]  ← MGLRU 回收入口
              └─ shrink_node()      [vmscan.c: ~7102]
                   └─ lru_gen_shrink_node()  [vmscan.c: ~6015]
                        ├─ shrink_many()    [vmscan.c: ~5924]  ← 多 memcg 路径
                        │    └─ shrink_one()
                        └─ shrink_one()     [vmscan.c: ~5883]  ← 单 memcg 路径
                             └─ try_to_shrink_lruvec()  [vmscan.c: ~5852]
                                  ├─ get_nr_to_scan()   [vmscan.c: ~5797]
                                  │    ├─ should_run_aging()       [vmscan.c: ~5732]
                                  │    └─ try_to_inc_max_seq()     [vmscan.c: ~4903]
                                  │         ├─ should_walk_mmu()
                                  │         ├─ iterate_mm_list()   [vmscan.c: ~3990]
                                  │         ├─ walk_mm()           [vmscan.c: ~4678]
                                  │         └─ inc_max_seq()       [vmscan.c: ~4835]
                                  └─ evict_folios()      [vmscan.c: ~5640]
                                       ├─ isolate_folios()         [vmscan.c: ~5593]
                                       │    └─ scan_folios()       [vmscan.c: ~5464]
                                       │       ├─ sort_folio()       [vmscan.c: ~5386]
                                       │       │   ├─ lru_gen_del_folio():uevitable [vmscan.c]
                                       │       │   └─ folio_inc_gen()       [vmscan.c: ~5386]
                                       │       └─ isolate_folio()       [vmscan.c: ~5464]
                                       │           └─ lru_gen_del_folio()       [vmscan.c: ~5386]
                                       ├─ try_to_inc_min_seq()
                                       ├─ shrink_folio_list()      [外部通用函数]
                                       └─ move_folios_to_lru()

 kswapd_run()                                    [mm/vmscan.c:9161]
    └─ kthread_run(kswapd, pgdat, ...)
         └─ kswapd()                              [mm/vmscan.c:9174]
              │  tsk->flags |= PF_MEMALLOC | PF_KSWAPD
              │  for (;;) {
              ├─ kswapd_try_to_sleep()            [mm/vmscan.c:9055]
              │    ├─ prepare_to_wait()           ← 设置 TASK_INTERRUPTIBLE
              │    ├─ wake_up_process(kcompactd)  ← 唤醒内存规整线程
              │    └─ schedule()                  ← 休眠，等待 wakeup_kswapd() 唤醒
              │
              ├─ kswapd_age_node
              │    │
              │    │ //MGLRU老化路径
              │    └─ lru_gen_age_node()
              │
              └─ balance_pgdat(pgdat, order, highest_zoneidx)  [mm/vmscan.c:8732]
                   │  sc.priority = DEF_PRIORITY  (初始优先级=12)
                   │  do {                        ← 优先级递减循环
                   │
                   ├─ [可选] memcg1_soft_limit_reclaim()   ← v1 软限制回收
                   │
                   ├─ kswapd_shrink_node(pgdat, &sc)        [mm/vmscan.c:8615]
                   │    │  sc->nr_to_reclaim = Σ max(high_wmark, SWAP_CLUSTER_MAX)
                   │    └─ shrink_node(pgdat, sc)            [mm/vmscan.c:7476]
                   │         │
                   │         │  ← MGLRU路径（MGLRU开启 且是 全局回收）
                   │         └─ lru_gen_shrink_node()
                   │              │
                   │              ├─ set_mm_walk()
                   │              ├─ set_initial_priority()
                   │              ├─ shrink_many()		// 使能MEMCG
                   │              │    └─ shrink_one()		// 逐个遍历memcg，对memcg lruvec进行老化和回收
                   │              ├─ shrink_one(&pgdat->__lruvec, sc)	// 没使能MEMCG，使用全局node的lruvec
		   │              │    │
		   │              │    │  //memcg内存限制判断，如果低于min，则不需要回收，将其放置到最新的位置（什么位置？）
		   │              │    │  if (mem_cgroup_below_min(NULL, memcg))
		   │              │    │      return MEMCG_LRU_YOUNG;
		   │              │    │
		   │              │    │  //如果低于low，再判断第二次机会是否用完，用完则继续回收
	           │              │    │  if (mem_cgroup_below_low(NULL, memcg))
		   │              │    │	   return MEMCG_LRU_TAIL;  //没用完，则降低放置到尾部位置，提前第二次机会
		   │              │    │      return MEMCG_LRU_YOUNG;
		   │              │    │
		   │              │    │
                   │              │    ├─ try_to_shrink_lruvec()	// 不断老化和回收，直到满足需求
		   │              │    │    │
		   │              │    │    │  ** 页面老化 **
                   │              │    │    ├─ get_nr_to_scan()
		   │              │    │    │    │
		   │              │    │    │    │  判断是否需要老化:
		   │              │    │    │    │	不需要老化: gen == 4
		   │              │    │    │    │	需要老化  : gen <= 1
		   │              │    │    │    │	需要老化  : gen ==3，但是热页过多或者冷页过少
                   │              │    │    │    ├─ should_run_aging()
		   │              │    │    │    │
                   │              │    │    │    └─ try_to_inc_max_seq()
                   │              │    │    │         ├─ should_walk_mmu()
                   │              │    │    │         ├─ iterate_mm_list()
		   │              │    │    │         │
		   │              │    │    │         │  ** 遍历memcg下的进程页表, 将asecess bit被置位的folio提升到最新的gen **
                   │              │    │    │         ├─ walk_mm()
		   │              │    │    │         │    └─ walk_page_range()
		   │              │    │    │         │         └─ walk_pud_range()
		   │              │    │    │         │              └─ walk_pmd_range()
		   │              │    │    │         │                    ├─ walk_pte_range()
		   │              │    │    │         │                    │    └─ folio_update_gen()←update gen 调用点1，将页面提升到最新gen（只改flags）
		   │              │    │    │         │                    │
		   │              │    │    │         │                    └─ walk_pmd_range_locked()
		   │              │    │    │         │                         └─ folio_update_gen()←update gen 调用点2
		   │              │    │    │         │
		   │              │    │    │         │ // 这里 update gen，后面哪里将其放到对应的gen? 
		   │              │    │    │         │		答：sort_folios根据gen，将页面提升
		   │              │    │    │         │ // MGLRU总是回收最老的gen，在哪里体现？上面walk_mm扫描的页面都是最老的gen吗
		   │              │    │    │         │		答：scan_folios中用lrugen->min_seq[type]获取最老gen的链表
		   │              │    │    │         │
		   │              │    │    │         │  ** 推进max seq，增加最新的gen **
                   │              │    │    │         └─ inc_max_seq()
		   │              │    │    │
		   │              │    │    │
		   │              │    │    │  ** 页面回收 **
                   │              │    │    └─ evict_folios()
                   │              │    │         ├─ isolate_folios()
		   │              │    │         │    │  //通过swappiness和min_seq判断应从哪一类页面开始回收
                   │              │    │         │    ├─ type = LRU_GEN_FILE or LRU_GEN_ANON
                   │              │    │         │    ├─ tier_idx = get_tier_idx()		//获取可以被回收的最大tier
		   │              │    │         │    │
		   │              │    │         │    │  // 根据type，从ANON开始遍历，
		   │              │    │         │    │  // type=FILE，只回收FILE，type=ANON，先回收ANON再回收FILE
		   │              │    │         │    │  // 从链表的尾部开始遍历
                   │              │    │         │    └─ scan_folios(type, tier_idx, list)
		   │              │    │         │         │
		   │              │    │         │         ├─ gen = lru_gen_from_seq(lrugen->min_seq[type]);	// 获取最老的gen
		   │              │    │         │         │
		   │              │    │         │         │  //遍历所有合适的zone
		   │              │    │         │         │  //对 &lrugen->folios[gen][type][zone]的所有页面调用sort_folio()进行筛选
		   │              │    │         │         │  //如果合适，则调用isolate_folio()隔离出来，如果隔离失败，则放回原来的链表
                   │              │    │         │         ├─ sort_folio(type, tier_idx)
		   │              │    │         │         │   │
                   │              │    │         │         │   ├─ lru_gen_del_folio()	// 如果是unevitable页面, 将其从原来的链表删除，然后返回
		   │              │    │         │         │   │
		   │              │    │         │         │   │  *** 老化后，在这里真正将页面移动到最新的gen的链表中 ***
		   │              │    │         │         │   │
		   │              │    │         │         │   │  //如果页面已经不是最老的gen，说明页面已经被异步提升了，则直接将其放入对应的MGLRU链表
		   │              │    │         │         │   │  if (gen != lru_gen_from_seq(lrugen->min_seq[type])) {
                   │              │    │         │         │   ├─ list_move(&folio->lru, &lrugen->folios[gen][type][zone]);
		   │              │    │         │         │   │
		   │              │    │         │         │   │
		   │              │    │         │         │   │  *** workingset页面保护， refault率判断 ***
		   │              │    │         │         │   │  //如果页面的tier大于tier_idx，或者当前页面的tier是最大的，则需要保护
		   │              │    │         │         │   │  //将页面提升一代，并加入到对应的链表尾部（较冷端）
		   │              │    │         │         │   │  if (tier > tier_idx || refs == BIT(LRU_REFS_WIDTH)) {
                   │              │    │         │         │   ├─ folio_inc_gen()
                   │              │    │         │         │   ├─ list_move_tail(&folio->lru, &lrugen->folios[gen][type][zone]);
		   │              │    │         │         │   │
		   │              │    │         │         │   │  ** 正在回写的页面也需要保护 **
		   │              │    │         │         │   │  //将页面提升一代，并加入到对应的链表头部（较热端）
                   │              │    │         │         │   ├─ folio_inc_gen()
                   │              │    │         │         │   └─list_move_tail(&folio->lru, &lrugen->folios[gen][type][zone]);
		   │              │    │         │         │
		   │              │    │         │         │  //页面被sort_folio()筛选通过，则尝试隔离
                   │              │    │         │         ├─ isolate_folio()
		   │              │    │         │         │    │  // 先判断能否被隔离（swap约束、被异步隔离过等）
                   │              │    │         │         │    └─ lru_gen_del_folio()		//如果能被隔离，先将页面从mglru链表移除
		   │              │    │         │         │
                   │              │    │         │         ├─ list_add(&folio->lru, list);	//页面可以被隔离，将页面加入list
                   │              │    │         │         └─ list_splice(&moved, head);	//将不可以被隔离的页面重新加到原来的链表
		   │              │    │         │						// 所以这里会使得最老gen里面还有页面
		   │              │    │         │
		   │              │    │         │  ** 尝试推进min_seq **
		   │              │    │         │    //如果最老gen在所有zone上都没有页面了，则可以推进，否则不推进
                   │              │    │         ├─ try_to_inc_min_seq()
		   │              │    │         │
		   │              │    │         │  ** 开始回收页面，和传统LRU流程一样 **
                   │              │    │         ├─ shrink_folio_list()      [外部通用函数]
                   │              │    │         │    │  逐个处理隔离出的 folio：
                   │              │    │         │    ├─ folio_trylock()
                   │              │    │         │    ├─ folio_check_references()        ← 引用检测
                   │              │    │         │    │    ├─ FOLIOREF_ACTIVATE  → 放回 active
                   │              │    │         │    │    ├─ FOLIOREF_KEEP      → 放回 inactive
                   │              │    │         │    │    └─ FOLIOREF_RECLAIM   → 继续回收
                   │              │    │         │    ├─ [匿名页] add_to_swap()  ← 分配 swap 空间
                   │              │    │         │    ├─ [mapped] try_to_unmap() ← 解除所有页表映射
                   │              │    │         │    ├─ [dirty] pageout()    ← 触发回写
                   │              │    │         │    │    └─ mapping->a_ops->writepage()
                   │              │    │         │    ├─ [clean] __remove_mapping() ← 从 page cache 删除
                   │              │    │         │    └─ free_unref_page() / 放入 free_pages list
                   │              │    │         └─ move_folios_to_lru()
		   │              │    │
		   │              │    └─ shrink_slab()
                   │              │
                   │              └─ clear_mm_walk()
                   │
                   │  } while (--sc->priority >= 0 && !pgdat_balanced())
                   │
                   └─ [balanced] 退出，kswapd 回到 kswapd_try_to_sleep()


** 文件页或者匿名页首次被放入LRU/MGLRU链表的流程 **
 page fault (do_anonymous_page / do_fault / filemap_fault 等)
    └─ folio_add_lru(folio) / folio_add_lru_vma(folio, vma)
         └─ folio_set_active(folio)   ← MGLRU特有：如果在 fault 路径中，标记为 active，MGLRU将新建页面加入最新的gen，传统LRU则是inactive链表
         └─ folio_batch_add_and_move(folio, lru_add, false)
                 └─ 加入 per-CPU folio_batch 缓存（lru_add batch）
                      └─ 缓存满 / 主动 drain 时：
                           folio_batch_move_lru(fbatch, lru_add)
                                └─ lru_add(lruvec, folio)
                                     └─ lruvec_add_folio(lruvec, folio)
                                          └─ lru_gen_add_folio(lruvec, folio, false)
                                               ← 真正加入 MGLRU/LRU 链表
 */
