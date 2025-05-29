// SPDX-License-Identifier: GPL-2.0
/*
 *  linux/fs/fat/cache.c
 *
 *  Written 1992,1993 by Werner Almesberger
 *
 *  Mar 1999. AV. Changed cache, so that it uses the starting cluster instead
 *	of inode number.
 *  May 1999. AV. Fixed the bogosity with FAT32 (read "FAT28"). Fscking lusers.
 */

#include <linux/slab.h>
#include "fat.h"

/* this must be > 0. */
#define FAT_MAX_CACHE	8

struct fat_cache {
	struct list_head cache_list;
	int nr_contig;	/* number of contiguous clusters */
	int fcluster;	/* cluster number in the file. */
	int dcluster;	/* cluster number on disk. */
};

struct fat_cache_id {
	unsigned int id;
	int nr_contig;
	int fcluster;
	int dcluster;
};

static inline int fat_max_cache(struct inode *inode)
{
	return FAT_MAX_CACHE;
}

static struct kmem_cache *fat_cache_cachep;

static void init_once(void *foo)
{
	struct fat_cache *cache = (struct fat_cache *)foo;

	INIT_LIST_HEAD(&cache->cache_list);
}

int __init fat_cache_init(void)
{
	fat_cache_cachep = kmem_cache_create("fat_cache",
				sizeof(struct fat_cache),
				0, SLAB_RECLAIM_ACCOUNT,
				init_once);
	if (fat_cache_cachep == NULL)
		return -ENOMEM;
	return 0;
}

void fat_cache_destroy(void)
{
	kmem_cache_destroy(fat_cache_cachep);
}

static inline struct fat_cache *fat_cache_alloc(struct inode *inode)
{
	return kmem_cache_alloc(fat_cache_cachep, GFP_NOFS);
}

static inline void fat_cache_free(struct fat_cache *cache)
{
	BUG_ON(!list_empty(&cache->cache_list));
	kmem_cache_free(fat_cache_cachep, cache);
}

static inline void fat_cache_update_lru(struct inode *inode,
					struct fat_cache *cache)
{
	if (MSDOS_I(inode)->cache_lru.next != &cache->cache_list)
		list_move(&cache->cache_list, &MSDOS_I(inode)->cache_lru);
}

static int fat_cache_lookup(struct inode *inode, int fclus,
			    struct fat_cache_id *cid,
			    int *cached_fclus, int *cached_dclus)
{
	static struct fat_cache nohit = { .fcluster = 0, };

	struct fat_cache *hit = &nohit, *p;
	int offset = -1;

	spin_lock(&MSDOS_I(inode)->cache_lru_lock);
	list_for_each_entry(p, &MSDOS_I(inode)->cache_lru, cache_list) {
		/* Find the cache of "fclus" or nearest cache. */
		if (p->fcluster <= fclus && hit->fcluster < p->fcluster) {
			hit = p;
			if ((hit->fcluster + hit->nr_contig) < fclus) {
				offset = hit->nr_contig;
			} else {
				offset = fclus - hit->fcluster;
				break;
			}
		}
	}
	if (hit != &nohit) {
		fat_cache_update_lru(inode, hit);

		cid->id = MSDOS_I(inode)->cache_valid_id;
		cid->nr_contig = hit->nr_contig;
		cid->fcluster = hit->fcluster;
		cid->dcluster = hit->dcluster;
		*cached_fclus = cid->fcluster + offset;
		*cached_dclus = cid->dcluster + offset;
	}
	spin_unlock(&MSDOS_I(inode)->cache_lru_lock);

	return offset;
}

static struct fat_cache *fat_cache_merge(struct inode *inode,
					 struct fat_cache_id *new)
{
	struct fat_cache *p;

	list_for_each_entry(p, &MSDOS_I(inode)->cache_lru, cache_list) {
		/* Find the same part as "new" in cluster-chain. */
		if (p->fcluster == new->fcluster) {
			BUG_ON(p->dcluster != new->dcluster);
			if (new->nr_contig > p->nr_contig)
				p->nr_contig = new->nr_contig;
			return p;
		}
	}
	return NULL;
}

static void fat_cache_add(struct inode *inode, struct fat_cache_id *new)
{
	struct fat_cache *cache, *tmp;

	if (new->fcluster == -1) /* dummy cache */
		return;

	spin_lock(&MSDOS_I(inode)->cache_lru_lock);
	if (new->id != FAT_CACHE_VALID &&
	    new->id != MSDOS_I(inode)->cache_valid_id)
		goto out;	/* this cache was invalidated */

	cache = fat_cache_merge(inode, new);
	if (cache == NULL) {
		if (MSDOS_I(inode)->nr_caches < fat_max_cache(inode)) {
			MSDOS_I(inode)->nr_caches++;
			spin_unlock(&MSDOS_I(inode)->cache_lru_lock);

			tmp = fat_cache_alloc(inode);
			if (!tmp) {
				spin_lock(&MSDOS_I(inode)->cache_lru_lock);
				MSDOS_I(inode)->nr_caches--;
				spin_unlock(&MSDOS_I(inode)->cache_lru_lock);
				return;
			}

			spin_lock(&MSDOS_I(inode)->cache_lru_lock);
			cache = fat_cache_merge(inode, new);
			if (cache != NULL) {
				MSDOS_I(inode)->nr_caches--;
				fat_cache_free(tmp);
				goto out_update_lru;
			}
			cache = tmp;
		} else {
			struct list_head *p = MSDOS_I(inode)->cache_lru.prev;
			cache = list_entry(p, struct fat_cache, cache_list);
		}
		cache->fcluster = new->fcluster;
		cache->dcluster = new->dcluster;
		cache->nr_contig = new->nr_contig;
	}
out_update_lru:
	fat_cache_update_lru(inode, cache);
out:
	spin_unlock(&MSDOS_I(inode)->cache_lru_lock);
}

/*
 * Cache invalidation occurs rarely, thus the LRU chain is not updated. It
 * fixes itself after a while.
 */
static void __fat_cache_inval_inode(struct inode *inode)
{
	struct msdos_inode_info *i = MSDOS_I(inode);
	struct fat_cache *cache;

	while (!list_empty(&i->cache_lru)) {
		cache = list_entry(i->cache_lru.next,
				   struct fat_cache, cache_list);
		list_del_init(&cache->cache_list);
		i->nr_caches--;
		fat_cache_free(cache);
	}
	/* Update. The copy of caches before this id is discarded. */
	i->cache_valid_id++;
	if (i->cache_valid_id == FAT_CACHE_VALID)
		i->cache_valid_id++;
}

void fat_cache_inval_inode(struct inode *inode)
{
	spin_lock(&MSDOS_I(inode)->cache_lru_lock);
	__fat_cache_inval_inode(inode);
	spin_unlock(&MSDOS_I(inode)->cache_lru_lock);
}

static inline int cache_contiguous(struct fat_cache_id *cid, int dclus)
{
	cid->nr_contig++;
	return ((cid->dcluster + cid->nr_contig) == dclus);
}

static inline void cache_init(struct fat_cache_id *cid, int fclus, int dclus)
{
	cid->id = FAT_CACHE_VALID;
	cid->fcluster = fclus;
	cid->dcluster = dclus;
	cid->nr_contig = 0;
}
/**
 * @brief 获取FAT文件系统中指定簇号的相关信息。
 *
 * 此函数负责根据inode和簇号，计算出该簇号在FAT表中的相关信息，
 * 包括当前簇号、目标簇号以及是否到达文件末尾等。
 *
 * @param inode 指向表示文件或目录的inode结构的指针。
 * @param cluster 要查询的逻辑簇号。
 * @param fclus 当前已经遍历过的簇号数量。
 * @param dclus 目标簇号。
 * @return 返回值可能为0（成功）、负数（错误）或FAT_ENT_EOF（到达文件末尾）。
 */
int fat_get_cluster(struct inode *inode, int cluster, int *fclus, int *dclus)
{
	struct super_block *sb = inode->i_sb;
	struct msdos_sb_info *sbi = MSDOS_SB(sb);
	const int limit = sb->s_maxbytes >> sbi->cluster_bits;	// 计算单个文件最大的逻辑簇号，fat32文件最大是4GB
	struct fat_entry fatent;	// fat表项
	struct fat_cache_id cid;
	int nr;

	/* 确保inode的起始簇号不为0，普通文件的起始簇号肯定不为0, */
	BUG_ON(MSDOS_I(inode)->i_start == 0);

	*fclus = 0;
	*dclus = MSDOS_I(inode)->i_start;
	if (!fat_valid_entry(sbi, *dclus)) {
		/*
		 * 物理起始簇号不在有效的范围内，可能的原因：
		 * 1.fat表或目录项损坏了
		 * 2.文件被删除，但inode信息仍然存在；
		 * 3.fat表和嗯目录项信息不匹配；
		 */
		fat_fs_error_ratelimit(sb,
			"%s: invalid start cluster (i_pos %lld, start %08x)",
			__func__, MSDOS_I(inode)->i_pos, *dclus);
		return -EIO;
	}
        /* ??: 如果簇号为0，直接返回0，表示起始簇号有效 */
	if (cluster == 0)
		return 0;

        /* 尝试从缓存中查找簇号信息，如果未命中，则初始化缓存 */
	if (fat_cache_lookup(inode, cluster, &cid, fclus, dclus) < 0) {
		/*
		 * dummy, always not contiguous
		 * This is reinitialized by cache_init(), later.
		 */
		cache_init(&cid, -1, -1);
	}

	fatent_init(&fatent);	//初始化fat表项
        /* 循环遍历簇链，直到找到目标簇号或到达文件末尾 */
	while (*fclus < cluster) {
		/* prevent the infinite loop of cluster chain */
                /* 防止无限循环，检查当前簇号是否超出文件系统的限制
		 * 遍历文件fat表链时，遍历文件逻辑簇号超过了最大的簇，那肯定有问题，形成了循环是什么意思？
		 * 可能的原因：
		 *	1.fat表损坏；
		 *	2.文件系统元数据不一致
		 */
		if (*fclus > limit) {
			fat_fs_error_ratelimit(sb,
				"%s: detected the cluster chain loop (i_pos %lld)",
				__func__, MSDOS_I(inode)->i_pos);
			nr = -EIO;
			goto out;
		}

                /* 读取FAT表项，获取下一个簇号 */
		nr = fat_ent_read(inode, &fatent, *dclus);
		if (nr < 0)	// 读取失败则退出
			goto out;
		/*
		 * 在遍历文件fat表链时，发现存在一个空闲的簇，说明fat表链中断了
		 * 可能是原因：
		 *	1.文件被部分删除？（有办法做到吗？截成两段？）
		 *	2.fat表被损坏了；
		 *	3.fat表和根目录项不匹配；
		 *	4.删除文件时，只删了fat表，根目录项信息还在？
		 *	5.文件系统被意外中断
		 */
		else if (nr == FAT_ENT_FREE) {
			fat_fs_error_ratelimit(sb,
				"%s: invalid cluster chain (i_pos %lld)",
				__func__, MSDOS_I(inode)->i_pos);
			nr = -EIO;
			goto out;
                /* 如果到达文件末尾，将缓存信息添加到缓存列表并退出循环 */
		} else if (nr == FAT_ENT_EOF) {
			fat_cache_add(inode, &cid);
			goto out;
		}
		(*fclus)++;
		*dclus = nr;	// 更新目标簇号为下一个簇号
                /* 如果当前簇号与缓存中的簇号不连续，则重新初始化缓存 */
		if (!cache_contiguous(&cid, *dclus))
			cache_init(&cid, *fclus, *dclus);
	}
	nr = 0;
	fat_cache_add(inode, &cid);	// 将缓存信息添加到缓存列表中
out:
	fatent_brelse(&fatent);
	return nr;
}

/**
 * @brief 获取FAT文件系统中指定簇号的实际物理簇号。
 *
 * 此函数用于根据inode和簇号，计算出该簇号在FAT表中的实际物理簇号。
 * 如果请求的簇号超出了文件的有效范围（即超出EOF），则会返回错误。
 *
 * @param inode 指向表示文件或目录的inode结构的指针。
 * @param cluster 要查询的逻辑簇号。
 * @return 实际的物理簇号或者错误码。
 */
/*
 * 获取文件在磁盘的物理起始簇号
 * cluster: 文件的逻辑簇号
 */
static int fat_bmap_cluster(struct inode *inode, int cluster)
{
	struct super_block *sb = inode->i_sb;
	int ret, fclus, dclus;

        /* 如果inode的起始簇号为0，则直接返回0，表示没有有效的数据 */
	if (MSDOS_I(inode)->i_start == 0)
		return 0;

        /* 调用fat_get_cluster函数获取指定簇号对应的物理簇号 */
	ret = fat_get_cluster(inode, cluster, &fclus, &dclus);
	if (ret < 0)
		return ret;
	else if (ret == FAT_ENT_EOF) {
                /* 如果请求的簇号超出了文件的有效范围（EOF）*/
		fat_fs_error(sb, "%s: request beyond EOF (i_pos %lld)",
			     __func__, MSDOS_I(inode)->i_pos);
		return -EIO;
	}
	return dclus;
}

/**
 * @brief 获取映射到FAT文件系统中的物理簇。
 *
 * 此函数负责将逻辑扇区号转换为FAT文件系统中的实际物理簇号，
 * 并计算出对应的偏移量以及连续映射块的数量。这是在处理文件数据时
 * 确定具体物理位置的重要步骤。
 *
 * @param inode 指向表示文件或目录的inode结构的指针。
 * @param sector 逻辑扇区号，需要映射到物理簇。
 * @param last_block 文件或分区中最后一个有效块的位置。
 * @param mapped_blocks 用于存储连续映射块数量的指针。
 * @param bmap 用于存储最终映射结果（物理块号）的指针。
 *
 * @return 成功返回0，或者如果映射失败则返回错误代码。
 */
int fat_get_mapped_cluster(struct inode *inode, sector_t sector,
			   sector_t last_block,
			   unsigned long *mapped_blocks, sector_t *bmap)
{
	struct super_block *sb = inode->i_sb;
	struct msdos_sb_info *sbi = MSDOS_SB(sb);
	int cluster, offset;

        /* 计算逻辑扇区所属的逻辑簇号 */
	cluster = sector >> (sbi->cluster_bits - sb->s_blocksize_bits);
        /* 计算扇区在簇内的偏移量 */
	offset  = sector & (sbi->sec_per_clus - 1);
        /* 获取逻辑簇的实际物理簇号 */
	cluster = fat_bmap_cluster(inode, cluster);
	if (cluster < 0)
		return cluster;
	else if (cluster) {
                /* 如果成功获取到物理簇号, 则进行如下计算 */
		*bmap = fat_clus_to_blknr(sbi, cluster) + offset;	// 将物理簇号转换为物理块号，并加上偏移量
		*mapped_blocks = sbi->sec_per_clus - offset;		// 计算从当前偏移开始还能映射多少个连续块
                /* 确保映射的块数不超过文件的有效范围 */
		if (*mapped_blocks > last_block - sector)
			*mapped_blocks = last_block - sector;
	}

	return 0;
}

static int is_exceed_eof(struct inode *inode, sector_t sector,
			 sector_t *last_block, int create)
{
	struct super_block *sb = inode->i_sb;
	const unsigned long blocksize = sb->s_blocksize;
	const unsigned char blocksize_bits = sb->s_blocksize_bits;

	*last_block = (i_size_read(inode) + (blocksize - 1)) >> blocksize_bits;
	if (sector >= *last_block) {
		if (!create)
			return 1;

		/*
		 * ->mmu_private can access on only allocation path.
		 * (caller must hold ->i_mutex)
		 */
		*last_block = (MSDOS_I(inode)->mmu_private + (blocksize - 1))
			>> blocksize_bits;
		if (sector >= *last_block)
			return 1;
	}

	return 0;
}

/**
 * @brief 将逻辑扇区映射到FAT文件系统中的物理扇区。
 *
 * 此函数负责确定给定逻辑扇区在FAT（文件分配表）文件系统中的物理位置。
 * 它处理数据扇区和目录项，确保基于文件系统的结构正确映射。
 *
 * @param inode 指向表示文件或目录的inode结构的指针。
 * @param sector 要映射的逻辑扇区号。
 * @param phys 用于存储结果物理扇区号的指针。
 * @param mapped_blocks 用于存储连续块数量的指针。
 * @param create 表示是否应在必要时分配新块。
 * @param from_bmap 表示调用是否源自bmap接口。
 *                  如果为真，则强制执行更严格的检查以确保扇区在范围内。
 *
 * @return 成功返回0，或者如果映射失败则返回错误代码。
 */
int fat_bmap(struct inode *inode, sector_t sector, sector_t *phys,
	     unsigned long *mapped_blocks, int create, bool from_bmap)
{
	struct msdos_sb_info *sbi = MSDOS_SB(inode->i_sb);	 // 获取超级块信息
	sector_t last_block;	// 用于存储文件的最后一个有效快

	*phys = 0;
	*mapped_blocks = 0;
        // 处理特殊情况：FAT12/16中的根目录。
	if (!is_fat32(sbi) && (inode->i_ino == MSDOS_ROOT_INO)) {
                // 如果不是FAT32且inode是根目录。
		if (sector < (sbi->dir_entries >> sbi->dir_per_block_bits)) {
                        // 如果逻辑扇区小于根目录项数除以每块目录项数。
			*phys = sector + sbi->dir_start;	// 计算物理扇区号
			*mapped_blocks = 1;		// 映射一个块
		}
		return 0;
	}

        // 根据调用来源判断是否需要超出EOF检查。
	if (!from_bmap) {
		if (is_exceed_eof(inode, sector, &last_block, create))	// 如果超出EOF，返回成功
			return 0;
	} else {
		// 计算文件的最后一个块
		last_block = inode->i_blocks >>
				(inode->i_sb->s_blocksize_bits - 9);
		if (sector >= last_block) //如果逻辑扇区超出文件范围，返回成功
			return 0;
	}

	// 进行簇映射
	return fat_get_mapped_cluster(inode, sector, last_block, mapped_blocks,
				      phys);
}
