/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_SWAP_H
#define _LINUX_SWAP_H

#include <linux/spinlock.h>
#include <linux/linkage.h>
#include <linux/mmzone.h>
#include <linux/list.h>
#include <linux/memcontrol.h>
#include <linux/sched.h>
#include <linux/node.h>
#include <linux/fs.h>
#include <linux/pagemap.h>
#include <linux/atomic.h>
#include <linux/page-flags.h>
#include <uapi/linux/mempolicy.h>
#include <asm/page.h>

struct notifier_block;

struct bio;

struct pagevec;

#define SWAP_FLAG_PREFER	0x8000	/* set if swap priority specified */
#define SWAP_FLAG_PRIO_MASK	0x7fff
#define SWAP_FLAG_PRIO_SHIFT	0
#define SWAP_FLAG_DISCARD	0x10000 /* enable discard for swap */
#define SWAP_FLAG_DISCARD_ONCE	0x20000 /* discard swap area at swapon-time */
#define SWAP_FLAG_DISCARD_PAGES 0x40000 /* discard page-clusters after use */

#define SWAP_FLAGS_VALID	(SWAP_FLAG_PRIO_MASK | SWAP_FLAG_PREFER | \
				 SWAP_FLAG_DISCARD | SWAP_FLAG_DISCARD_ONCE | \
				 SWAP_FLAG_DISCARD_PAGES)
#define SWAP_BATCH 64

static inline int current_is_kswapd(void)
{
	return current->flags & PF_KSWAPD;
}

/*
 * MAX_SWAPFILES defines the maximum number of swaptypes: things which can
 * be swapped to.  The swap type and the offset into that swap type are
 * encoded into pte's and into pgoff_t's in the swapcache.  Using five bits
 * for the type means that the maximum number of swapcache pages is 27 bits
 * on 32-bit-pgoff_t architectures.  And that assumes that the architecture packs
 * the type/offset into the pte as 5/27 as well.
 */
#define MAX_SWAPFILES_SHIFT	5

/*
 * Use some of the swap files numbers for other purposes. This
 * is a convenient way to hook into the VM to trigger special
 * actions on faults.
 */

/*
 * PTE markers are used to persist information onto PTEs that otherwise
 * should be a none pte.  As its name "PTE" hints, it should only be
 * applied to the leaves of pgtables.
 */
#define SWP_PTE_MARKER_NUM 1
#define SWP_PTE_MARKER     (MAX_SWAPFILES + SWP_HWPOISON_NUM + \
			    SWP_MIGRATION_NUM + SWP_DEVICE_NUM)

/*
 * Unaddressable device memory support. See include/linux/hmm.h and
 * Documentation/mm/hmm.rst. Short description is we need struct pages for
 * device memory that is unaddressable (inaccessible) by CPU, so that we can
 * migrate part of a process memory to device memory.
 *
 * When a page is migrated from CPU to device, we set the CPU page table entry
 * to a special SWP_DEVICE_{READ|WRITE} entry.
 *
 * When a page is mapped by the device for exclusive access we set the CPU page
 * table entries to special SWP_DEVICE_EXCLUSIVE_* entries.
 */
#ifdef CONFIG_DEVICE_PRIVATE
#define SWP_DEVICE_NUM 4
#define SWP_DEVICE_WRITE (MAX_SWAPFILES+SWP_HWPOISON_NUM+SWP_MIGRATION_NUM)
#define SWP_DEVICE_READ (MAX_SWAPFILES+SWP_HWPOISON_NUM+SWP_MIGRATION_NUM+1)
#define SWP_DEVICE_EXCLUSIVE_WRITE (MAX_SWAPFILES+SWP_HWPOISON_NUM+SWP_MIGRATION_NUM+2)
#define SWP_DEVICE_EXCLUSIVE_READ (MAX_SWAPFILES+SWP_HWPOISON_NUM+SWP_MIGRATION_NUM+3)
#else
#define SWP_DEVICE_NUM 0
#endif

/*
 * Page migration support.
 *
 * SWP_MIGRATION_READ_EXCLUSIVE is only applicable to anonymous pages and
 * indicates that the referenced (part of) an anonymous page is exclusive to
 * a single process. For SWP_MIGRATION_WRITE, that information is implicit:
 * (part of) an anonymous page that are mapped writable are exclusive to a
 * single process.
 */
#ifdef CONFIG_MIGRATION
#define SWP_MIGRATION_NUM 3
#define SWP_MIGRATION_READ (MAX_SWAPFILES + SWP_HWPOISON_NUM)
#define SWP_MIGRATION_READ_EXCLUSIVE (MAX_SWAPFILES + SWP_HWPOISON_NUM + 1)
#define SWP_MIGRATION_WRITE (MAX_SWAPFILES + SWP_HWPOISON_NUM + 2)
#else
#define SWP_MIGRATION_NUM 0
#endif

/*
 * Handling of hardware poisoned pages with memory corruption.
 */
#ifdef CONFIG_MEMORY_FAILURE
#define SWP_HWPOISON_NUM 1
#define SWP_HWPOISON		MAX_SWAPFILES
#else
#define SWP_HWPOISON_NUM 0
#endif

#define MAX_SWAPFILES \
	((1 << MAX_SWAPFILES_SHIFT) - SWP_DEVICE_NUM - \
	SWP_MIGRATION_NUM - SWP_HWPOISON_NUM - \
	SWP_PTE_MARKER_NUM)

/*
 * Magic header for a swap area. The first part of the union is
 * what the swap magic looks like for the old (limited to 128MB)
 * swap area format, the second part of the union adds - in the
 * old reserved area - some extra information. Note that the first
 * kilobyte is reserved for boot loader or disk label stuff...
 *
 * Having the magic at the end of the PAGE_SIZE makes detecting swap
 * areas somewhat tricky on machines that support multiple page sizes.
 * For 2.5 we'll probably want to move the magic to just beyond the
 * bootbits...
 */
union swap_header {
	struct {
		char reserved[PAGE_SIZE - 10];
		char magic[10];			/* SWAP-SPACE or SWAPSPACE2 */
	} magic;
	struct {
		char		bootbits[1024];	/* Space for disklabel etc. */
		__u32		version;
		__u32		last_page;
		__u32		nr_badpages;
		unsigned char	sws_uuid[16];
		unsigned char	sws_volume[16];
		__u32		padding[117];
		__u32		badpages[1];
	} info;
};

/*
 * current->reclaim_state points to one of these when a task is running
 * memory reclaim
 */
struct reclaim_state {
	/* pages reclaimed outside of LRU-based reclaim */
	unsigned long reclaimed;
#ifdef CONFIG_LRU_GEN
	/* per-thread mm walk data */
	struct lru_gen_mm_walk *mm_walk;
#endif
};

/*
 * mm_account_reclaimed_pages(): account reclaimed pages outside of LRU-based
 * reclaim
 * @pages: number of pages reclaimed
 *
 * If the current process is undergoing a reclaim operation, increment the
 * number of reclaimed pages by @pages.
 */
static inline void mm_account_reclaimed_pages(unsigned long pages)
{
	if (current->reclaim_state)
		current->reclaim_state->reclaimed += pages;
}

#ifdef __KERNEL__

struct address_space;
struct sysinfo;
struct writeback_control;
struct zone;

/*
 * A swap extent maps a range of a swapfile's PAGE_SIZE pages onto a range of
 * disk blocks.  A rbtree of swap extents maps the entire swapfile (Where the
 * term `swapfile' refers to either a blockdevice or an IS_REG file). Apart
 * from setup, they're handled identically.
 *
 * We always assume that blocks are of size PAGE_SIZE.
 */
/*
 * swap_extent用于管理交换文件和swap分区对应磁盘的映射关系
 * 因为交换文件可能是部分swap, 或者存储到磁盘时是碎片化存储，例如
 *
 * 交换文件逻辑空间（4KB页面）：
 *	页面 0-99    → 磁盘块 1000-1099（连续）
 *	页面 100-199 → 磁盘块 2000-2099（不连续，跳跃了）
 *	页面 200-299 → 磁盘块 5000-5099（又不连续
 *
 *        swap_extent_root
 *             |
 *       [se1: 0-99]
 *          /    \
 *         /      \
 *  [se2: 100-199] [se3: 200-299]
 *
 *	se1: {start_page=0, nr_pages=100, start_block=1000}
 *	se2: {start_page=100, nr_pages=100, start_block=2000}
 *	se3: {start_page=200, nr_pages=100, start_block=5000}
 *
 * zram不用swap_extent，因为zram是直接使用简单的线性数组 zram->table[]
 */
struct swap_extent {
	struct rb_node rb_node;
	pgoff_t start_page;	// 文件起始页面（在交换文件中的逻辑偏移）
	pgoff_t nr_pages;	// 连续的页面数量
	sector_t start_block;	// 起始块号(磁盘的起始物理位置)
};

/*
 * Max bad pages in the new format..
 */
#define MAX_SWAP_BADPAGES \
	((offsetof(union swap_header, magic.magic) - \
	  offsetof(union swap_header, info.badpages)) / sizeof(int))

enum {
	SWP_USED	= (1 << 0),	/* is slot in swap_info[] used? */
	SWP_WRITEOK	= (1 << 1),	/* ok to write to this swap?	*/
	SWP_DISCARDABLE = (1 << 2),	/* blkdev support discard */
	SWP_DISCARDING	= (1 << 3),	/* now discarding a free cluster */
	SWP_SOLIDSTATE	= (1 << 4),	/* blkdev seeks are cheap */
	SWP_CONTINUED	= (1 << 5),	/* swap_map has count continuation */
	SWP_BLKDEV	= (1 << 6),	/* its a block device */
	SWP_ACTIVATED	= (1 << 7),	/* set after swap_activate success */
	SWP_FS_OPS	= (1 << 8),	/* swapfile operations go through fs */
	SWP_AREA_DISCARD = (1 << 9),	/* single-time swap area discards */
	SWP_PAGE_DISCARD = (1 << 10),	/* freed swap page-cluster discards */
	SWP_STABLE_WRITES = (1 << 11),	/* no overwrite PG_writeback pages */
	SWP_SYNCHRONOUS_IO = (1 << 12),	/* synchronous IO is efficient */
					/* add others here before... */
	SWP_SCANNING	= (1 << 14),	/* refcount in scan_swap_map */
};

#define SWAP_CLUSTER_MAX 32UL
#define COMPACT_CLUSTER_MAX SWAP_CLUSTER_MAX

/* Bit flag in swap_map */
#define SWAP_HAS_CACHE	0x40	/* Flag page is cached, in first swap_map */
#define COUNT_CONTINUED	0x80	/* Flag swap_map continuation for full count */

/* Special value in first swap_map */
#define SWAP_MAP_MAX	0x3e	/* Max count */
#define SWAP_MAP_BAD	0x3f	/* Note page is bad */
#define SWAP_MAP_SHMEM	0xbf	/* Owned by shmem/tmpfs */

/* Special value in each swap_map continuation */
#define SWAP_CONT_MAX	0x7f	/* Max count */

/*
 * We use this to track usage of a cluster. A cluster is a block of swap disk
 * space with SWAPFILE_CLUSTER pages long and naturally aligns in disk. All
 * free clusters are organized into a list. We fetch an entry from the list to
 * get a free cluster.
 *
 * The flags field determines if a cluster is free. This is
 * protected by cluster lock.
 */
/*
 * 我们知道swap_info_struct中的swap_map用于记录每个swap slot的使用情况，可以通过查询swap_map来寻找空闲的slot；
 * 但如果系统同一时间有多个进程同进Swap操作，那么就会同时访问 swap_map[] 数组，从而产生资源竞争，
 * 就需要全局的 spin_lock 来保护，在这种情况下如果系统中有 n个 cpu 同时访问这个数组，那么只能一个一个地访问，
 * 从而阻塞了其它 cpu 的执行，而且随着CPU核心数增加，锁竞争会加剧。基于这个问题，把所有的页槽组织成以 256个
 * 页槽为一个簇的链表，这样不同CPU可以同时访问不同的簇。
 *
 * 和zram不用swap_extent一样，zram也不用swap_cluster_info，因为zram是基于内存的，随机访问极快，无需磁盘优化策略;
 * zram用的是内存中的zram->table[index]线性数组进行访问, 例如zram_get_handle(zram, index),
 * 直接使用index从数组中获取zs_malloc分配的空间地址.
 */
struct swap_cluster_info {
	spinlock_t lock;	/*
				 * Protect swap_cluster_info fields
				 * other than list, and swap_info_struct->swap_map
				 * elements corresponding to the swap cluster.
				 */
	u16 count;		// 当前cluster中已使用的页面数
	u8 flags;		// cluster状态标志，具体见下面的CLUSTER_FLAG_*
	u8 order;		// cluster大小等级，用于支持大页
	struct list_head list;	// 链表节点，根据状态加入不同类型的链表，见下面的CLUSTER_FLAG_*
};

/*
 * 三个主要的链表:
 * free_clusters     → [集群A] → [集群B] → [集群C] → ... (所有页面空闲)
 * nonfull_clusters  → [集群D] → [集群E] → [集群F] → ... (部分页面使用)
 * full_clusters     → [集群G] → [集群H] → [集群I] → ... (全部页面使用)
 *
 * 示例:
 * +-------------------+
 * | lock: 自旋锁      |
 * | count: 5/256      | ← 已使用5个页面，总共256个
 * | flags: NONFULL(2) | ← 非满状态
 * | order: 0          | ← 标准大小（256页面）
 * | list: [链表节点]  | ← 在nonfull_clusters链表中
 * +-------------------+
 *
 * swap_cluster初始化接口： setup_clusters
 *
 * 场景：频繁分配释放导致碎片化
 * 传统：swap_map: [1,0,1,0,1,0,1,0,1,0,...] 交替使用
 *     每次分配都要线性扫描
 *
 * 集群：将256页面作为单元管理
 *     只要集群内有空闲页面，就从nonfull链表快速分配
 *     减少全局搜索开销
 *
 */
#define CLUSTER_FLAG_FREE 1 /* This cluster is free */			// 完全空闲的cluster(集群)
#define CLUSTER_FLAG_NONFULL 2 /* This cluster is on nonfull list */	// 部分使用的cluster
#define CLUSTER_FLAG_FRAG 4 /* This cluster is on nonfull list */	// nonfull，碎片化的cluster
#define CLUSTER_FLAG_FULL 8 /* This cluster is on full list */		// 已经用满的cluster

/*
 * The first page in the swap file is the swap header, which is always marked
 * bad to prevent it from being allocated as an entry. This also prevents the
 * cluster to which it belongs being marked free. Therefore 0 is safe to use as
 * a sentinel to indicate next is not valid in percpu_cluster.
 */
#define SWAP_NEXT_INVALID	0

#ifdef CONFIG_THP_SWAP
#define SWAP_NR_ORDERS		(PMD_ORDER + 1)
#else
#define SWAP_NR_ORDERS		1
#endif

/*
 * We assign a cluster to each CPU, so each CPU can allocate swap entry from
 * its own cluster and swapout sequentially. The purpose is to optimize swapout
 * throughput.
 */
struct percpu_cluster {
	unsigned int next[SWAP_NR_ORDERS]; /* Likely next allocation offset */
};

/*
 * The in-memory structure used to track swap areas.
 */
/*
 * swap_info_struct用于管理swap分区
 * 每个swap分区都由一个si结构体管理
 */
struct swap_info_struct {
	struct percpu_ref users;	/* indicate and keep swap device valid. */
	unsigned long	flags;		/* SWP_USED etc: see above */
	/* 这个swap分区的优先级, swap分区初始化时确定(setup_swap_info)，查找时按优先级高低查找对应槽位，how? 查找什么？*/
	/* 在get_swap_pages中，按照优先级从高到低(prio数值越低，优先级越高)，从高优先级的swap分区开始查找空闲的slot */
	signed short	prio;		/* swap priority of this type */
	struct plist_node list;		/* entry in swap_active_head */
	/*
	 * 其实是index, 表示第几个swap分区，第一个创建的swap分区，type为0
	 * 该变量同时也是 swap_info[] 全局变量的索引，即swap_info[0] 就是第一个 swap 分区的 swap_info_struct 结构体。
	 * swap_info[] 数组保存着系统中所有swap 分区描述符，即 swap_info_struct 结构体
	 */
	signed char	type;		/* strange name for an index */
	unsigned int	max;		/* extent of the swap_map */
	/*
	 * slot数组，用于统计swap分区每个slot的空闲情况/引用个数，一般是一个页面对应一个数组成员
	 * 0表示slot空闲，n(>0)表示该页面被映射的进程数目
	 */
	unsigned char *swap_map;	/* vmalloc'ed array of usage counts */
	unsigned long *zeromap;		/* kvmalloc'ed bitmap to track zero pages */
	struct swap_cluster_info *cluster_info; /* cluster info. Only for SSD */
	struct list_head free_clusters; /* free clusters list */
	struct list_head full_clusters; /* full clusters list */
	struct list_head nonfull_clusters[SWAP_NR_ORDERS];
					/* list of cluster that contains at least one free slot */
	struct list_head frag_clusters[SWAP_NR_ORDERS];
					/* list of cluster that are fragmented or contented */
	unsigned int frag_cluster_nr[SWAP_NR_ORDERS];
	unsigned int lowest_bit;	/* index of first free in swap_map */
	unsigned int highest_bit;	/* index of last free in swap_map */
	/*
	 * swap分区可用的总页面数量
	 * 如果是传统磁盘分区，要除去坏块; 如果是zram，等于zram分区大小/页面大小
	 */
	unsigned int pages;		/* total of usable pages of swap */
	/*
	 * 已经被swapout的页面数量
	 * 比如有10个页面被压缩到zram分区，那inuse_pages即为10
	 */
	unsigned int inuse_pages;	/* number of those currently in use */
	unsigned int cluster_next;	/* likely index for next allocation */
	unsigned int cluster_nr;	/* countdown to next cluster search */
	unsigned int __percpu *cluster_next_cpu; /*percpu index for next allocation */
	struct percpu_cluster __percpu *percpu_cluster; /* per cpu's swap location */
	struct rb_root swap_extent_root;/* root of the swap extent rbtree */
	/* swap 分区所处于的块设备 struct block_device 结构体 */
	struct block_device *bdev;	/* swap device or bdev of swap file */
	/* swap分区对应的swap_file结构体 */
	struct file *swap_file;		/* seldom referenced */
	struct completion comp;		/* seldom referenced */
	spinlock_t lock;		/*
					 * protect map scan related fields like
					 * swap_map, lowest_bit, highest_bit,
					 * inuse_pages, cluster_next,
					 * cluster_nr, lowest_alloc,
					 * highest_alloc, free/discard cluster
					 * list. other fields are only changed
					 * at swapon/swapoff, so are protected
					 * by swap_lock. changing flags need
					 * hold this lock and swap_lock. If
					 * both locks need hold, hold swap_lock
					 * first.
					 */
	spinlock_t cont_lock;		/*
					 * protect swap count continuation page
					 * list.
					 */
	struct work_struct discard_work; /* discard worker */
	struct work_struct reclaim_work; /* reclaim worker */
	struct list_head discard_clusters; /* discard clusters list */
	struct plist_node avail_lists[]; /*
					   * entries in swap_avail_heads, one
					   * entry per node.
					   * Must be last as the number of the
					   * array is nr_node_ids, which is not
					   * a fixed value so have to allocate
					   * dynamically.
					   * And it has to be an array so that
					   * plist_for_each_* can work.
					   */
};

static inline swp_entry_t page_swap_entry(struct page *page)
{
	struct folio *folio = page_folio(page);
	swp_entry_t entry = folio->swap;

	entry.val += folio_page_idx(folio, page);
	return entry;
}

/* linux/mm/workingset.c */
bool workingset_test_recent(void *shadow, bool file, bool *workingset,
				bool flush);
void workingset_age_nonresident(struct lruvec *lruvec, unsigned long nr_pages);
void *workingset_eviction(struct folio *folio, struct mem_cgroup *target_memcg);
void workingset_refault(struct folio *folio, void *shadow);
void workingset_activation(struct folio *folio);

/* linux/mm/page_alloc.c */
extern unsigned long totalreserve_pages;

/* Definition of global_zone_page_state not available yet */
#define nr_free_pages() global_zone_page_state(NR_FREE_PAGES)


/* linux/mm/swap.c */
void lru_note_cost(struct lruvec *lruvec, bool file,
		   unsigned int nr_io, unsigned int nr_rotated);
void lru_note_cost_refault(struct folio *);
void folio_add_lru(struct folio *);
void folio_add_lru_vma(struct folio *, struct vm_area_struct *);
void mark_page_accessed(struct page *);
void folio_mark_accessed(struct folio *);

extern atomic_t lru_disable_count;

static inline bool lru_cache_disabled(void)
{
	return atomic_read(&lru_disable_count);
}

static inline void lru_cache_enable(void)
{
	atomic_dec(&lru_disable_count);
}

extern void lru_cache_disable(void);
extern void lru_add_drain(void);
extern void lru_add_drain_cpu(int cpu);
extern void lru_add_drain_cpu_zone(struct zone *zone);
extern void lru_add_drain_all(void);
void folio_deactivate(struct folio *folio);
void folio_mark_lazyfree(struct folio *folio);
extern void swap_setup(void);

/* linux/mm/vmscan.c */
extern unsigned long zone_reclaimable_pages(struct zone *zone);
extern unsigned long try_to_free_pages(struct zonelist *zonelist, int order,
					gfp_t gfp_mask, nodemask_t *mask);

#define MEMCG_RECLAIM_MAY_SWAP (1 << 1)
#define MEMCG_RECLAIM_PROACTIVE (1 << 2)
#define MIN_SWAPPINESS 0
#define MAX_SWAPPINESS 200
extern unsigned long try_to_free_mem_cgroup_pages(struct mem_cgroup *memcg,
						  unsigned long nr_pages,
						  gfp_t gfp_mask,
						  unsigned int reclaim_options,
						  int *swappiness);
extern unsigned long mem_cgroup_shrink_node(struct mem_cgroup *mem,
						gfp_t gfp_mask, bool noswap,
						pg_data_t *pgdat,
						unsigned long *nr_scanned);
extern unsigned long shrink_all_memory(unsigned long nr_pages);
extern int vm_swappiness;
long remove_mapping(struct address_space *mapping, struct folio *folio);

#ifdef CONFIG_NUMA
extern int node_reclaim_mode;
extern int sysctl_min_unmapped_ratio;
extern int sysctl_min_slab_ratio;
#else
#define node_reclaim_mode 0
#endif

static inline bool node_reclaim_enabled(void)
{
	/* Is any node_reclaim_mode bit set? */
	return node_reclaim_mode & (RECLAIM_ZONE|RECLAIM_WRITE|RECLAIM_UNMAP);
}

void check_move_unevictable_folios(struct folio_batch *fbatch);

extern void __meminit kswapd_run(int nid);
extern void __meminit kswapd_stop(int nid);

#ifdef CONFIG_SWAP

int add_swap_extent(struct swap_info_struct *sis, unsigned long start_page,
		unsigned long nr_pages, sector_t start_block);
int generic_swapfile_activate(struct swap_info_struct *, struct file *,
		sector_t *);

static inline unsigned long total_swapcache_pages(void)
{
	return global_node_page_state(NR_SWAPCACHE);
}

void free_swap_cache(struct folio *folio);
void free_page_and_swap_cache(struct page *);
void free_pages_and_swap_cache(struct encoded_page **, int);
/* linux/mm/swapfile.c */
extern atomic_long_t nr_swap_pages;
extern long total_swap_pages;
extern atomic_t nr_rotate_swap;
extern bool has_usable_swap(void);

/* Swap 50% full? Release swapcache more aggressively.. */
static inline bool vm_swap_full(void)
{
	return atomic_long_read(&nr_swap_pages) * 2 < total_swap_pages;
}

static inline long get_nr_swap_pages(void)
{
	return atomic_long_read(&nr_swap_pages);
}

extern void si_swapinfo(struct sysinfo *);
swp_entry_t folio_alloc_swap(struct folio *folio);
bool folio_free_swap(struct folio *folio);
void put_swap_folio(struct folio *folio, swp_entry_t entry);
extern swp_entry_t get_swap_page_of_type(int);
extern int get_swap_pages(int n, swp_entry_t swp_entries[], int order);
extern int add_swap_count_continuation(swp_entry_t, gfp_t);
extern void swap_shmem_alloc(swp_entry_t, int);
extern int swap_duplicate(swp_entry_t);
extern int swapcache_prepare(swp_entry_t entry, int nr);
extern void swap_free_nr(swp_entry_t entry, int nr_pages);
extern void swapcache_free_entries(swp_entry_t *entries, int n);
extern void free_swap_and_cache_nr(swp_entry_t entry, int nr);
int swap_type_of(dev_t device, sector_t offset);
int find_first_swap(dev_t *device);
extern unsigned int count_swap_pages(int, int);
extern sector_t swapdev_block(int, pgoff_t);
extern int __swap_count(swp_entry_t entry);
extern int swap_swapcount(struct swap_info_struct *si, swp_entry_t entry);
extern int swp_swapcount(swp_entry_t entry);
struct swap_info_struct *swp_swap_info(swp_entry_t entry);
struct backing_dev_info;
extern int init_swap_address_space(unsigned int type, unsigned long nr_pages);
extern void exit_swap_address_space(unsigned int type);
extern struct swap_info_struct *get_swap_device(swp_entry_t entry);
sector_t swap_folio_sector(struct folio *folio);

static inline void put_swap_device(struct swap_info_struct *si)
{
	percpu_ref_put(&si->users);
}

#else /* CONFIG_SWAP */
static inline struct swap_info_struct *swp_swap_info(swp_entry_t entry)
{
	return NULL;
}

static inline struct swap_info_struct *get_swap_device(swp_entry_t entry)
{
	return NULL;
}

static inline void put_swap_device(struct swap_info_struct *si)
{
}

#define get_nr_swap_pages()			0L
#define total_swap_pages			0L
#define total_swapcache_pages()			0UL
#define vm_swap_full()				0

#define si_swapinfo(val) \
	do { (val)->freeswap = (val)->totalswap = 0; } while (0)
/* only sparc can not include linux/pagemap.h in this file
 * so leave put_page and release_pages undeclared... */
#define free_page_and_swap_cache(page) \
	put_page(page)
#define free_pages_and_swap_cache(pages, nr) \
	release_pages((pages), (nr));

static inline void free_swap_and_cache_nr(swp_entry_t entry, int nr)
{
}

static inline void free_swap_cache(struct folio *folio)
{
}

static inline int add_swap_count_continuation(swp_entry_t swp, gfp_t gfp_mask)
{
	return 0;
}

static inline void swap_shmem_alloc(swp_entry_t swp, int nr)
{
}

static inline int swap_duplicate(swp_entry_t swp)
{
	return 0;
}

static inline int swapcache_prepare(swp_entry_t swp, int nr)
{
	return 0;
}

static inline void swap_free_nr(swp_entry_t entry, int nr_pages)
{
}

static inline void put_swap_folio(struct folio *folio, swp_entry_t swp)
{
}

static inline int __swap_count(swp_entry_t entry)
{
	return 0;
}

static inline int swap_swapcount(struct swap_info_struct *si, swp_entry_t entry)
{
	return 0;
}

static inline int swp_swapcount(swp_entry_t entry)
{
	return 0;
}

static inline swp_entry_t folio_alloc_swap(struct folio *folio)
{
	swp_entry_t entry;
	entry.val = 0;
	return entry;
}

static inline bool folio_free_swap(struct folio *folio)
{
	return false;
}

static inline int add_swap_extent(struct swap_info_struct *sis,
				  unsigned long start_page,
				  unsigned long nr_pages, sector_t start_block)
{
	return -EINVAL;
}
#endif /* CONFIG_SWAP */

static inline void free_swap_and_cache(swp_entry_t entry)
{
	free_swap_and_cache_nr(entry, 1);
}

static inline void swap_free(swp_entry_t entry)
{
	swap_free_nr(entry, 1);
}

#ifdef CONFIG_MEMCG
static inline int mem_cgroup_swappiness(struct mem_cgroup *memcg)
{
	/* Cgroup2 doesn't have per-cgroup swappiness */
	if (cgroup_subsys_on_dfl(memory_cgrp_subsys))
		return READ_ONCE(vm_swappiness);

	/* root ? */
	if (mem_cgroup_disabled() || mem_cgroup_is_root(memcg))
		return READ_ONCE(vm_swappiness);

	return READ_ONCE(memcg->swappiness);
}
#else
static inline int mem_cgroup_swappiness(struct mem_cgroup *mem)
{
	return READ_ONCE(vm_swappiness);
}
#endif

#if defined(CONFIG_SWAP) && defined(CONFIG_MEMCG) && defined(CONFIG_BLK_CGROUP)
void __folio_throttle_swaprate(struct folio *folio, gfp_t gfp);
static inline void folio_throttle_swaprate(struct folio *folio, gfp_t gfp)
{
	if (mem_cgroup_disabled())
		return;
	__folio_throttle_swaprate(folio, gfp);
}
#else
static inline void folio_throttle_swaprate(struct folio *folio, gfp_t gfp)
{
}
#endif

#if defined(CONFIG_MEMCG) && defined(CONFIG_SWAP)
void mem_cgroup_swapout(struct folio *folio, swp_entry_t entry);
int __mem_cgroup_try_charge_swap(struct folio *folio, swp_entry_t entry);
static inline int mem_cgroup_try_charge_swap(struct folio *folio,
		swp_entry_t entry)
{
	if (mem_cgroup_disabled())
		return 0;
	return __mem_cgroup_try_charge_swap(folio, entry);
}

extern void __mem_cgroup_uncharge_swap(swp_entry_t entry, unsigned int nr_pages);
static inline void mem_cgroup_uncharge_swap(swp_entry_t entry, unsigned int nr_pages)
{
	if (mem_cgroup_disabled())
		return;
	__mem_cgroup_uncharge_swap(entry, nr_pages);
}

extern long mem_cgroup_get_nr_swap_pages(struct mem_cgroup *memcg);
extern bool mem_cgroup_swap_full(struct folio *folio);
#else
static inline void mem_cgroup_swapout(struct folio *folio, swp_entry_t entry)
{
}

static inline int mem_cgroup_try_charge_swap(struct folio *folio,
					     swp_entry_t entry)
{
	return 0;
}

static inline void mem_cgroup_uncharge_swap(swp_entry_t entry,
					    unsigned int nr_pages)
{
}

static inline long mem_cgroup_get_nr_swap_pages(struct mem_cgroup *memcg)
{
	return get_nr_swap_pages();
}

static inline bool mem_cgroup_swap_full(struct folio *folio)
{
	return vm_swap_full();
}
#endif

#endif /* __KERNEL__*/
#endif /* _LINUX_SWAP_H */
