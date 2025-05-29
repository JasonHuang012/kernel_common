// SPDX-License-Identifier: GPL-2.0-only
/*
 *  linux/fs/fat/file.c
 *
 *  Written 1992,1993 by Werner Almesberger
 *
 *  regular file handling primitives for fat-based filesystems
 */

#include <linux/capability.h>
#include <linux/module.h>
#include <linux/compat.h>
#include <linux/mount.h>
#include <linux/blkdev.h>
#include <linux/backing-dev.h>
#include <linux/fsnotify.h>
#include <linux/security.h>
#include <linux/falloc.h>
#include "fat.h"

static long fat_fallocate(struct file *file, int mode,
			  loff_t offset, loff_t len);

static int fat_ioctl_get_attributes(struct inode *inode, u32 __user *user_attr)
{
	u32 attr;

	inode_lock_shared(inode);
	attr = fat_make_attrs(inode);
	inode_unlock_shared(inode);

	return put_user(attr, user_attr);
}

static int fat_ioctl_set_attributes(struct file *file, u32 __user *user_attr)
{
	struct inode *inode = file_inode(file);
	struct msdos_sb_info *sbi = MSDOS_SB(inode->i_sb);
	int is_dir = S_ISDIR(inode->i_mode);
	u32 attr, oldattr;
	struct iattr ia;
	int err;

	err = get_user(attr, user_attr);
	if (err)
		goto out;

	err = mnt_want_write_file(file);
	if (err)
		goto out;
	inode_lock(inode);

	/*
	 * ATTR_VOLUME and ATTR_DIR cannot be changed; this also
	 * prevents the user from turning us into a VFAT
	 * longname entry.  Also, we obviously can't set
	 * any of the NTFS attributes in the high 24 bits.
	 */
	attr &= 0xff & ~(ATTR_VOLUME | ATTR_DIR);
	/* Merge in ATTR_VOLUME and ATTR_DIR */
	attr |= (MSDOS_I(inode)->i_attrs & ATTR_VOLUME) |
		(is_dir ? ATTR_DIR : 0);
	oldattr = fat_make_attrs(inode);

	/* Equivalent to a chmod() */
	ia.ia_valid = ATTR_MODE | ATTR_CTIME;
	ia.ia_ctime = current_time(inode);
	if (is_dir)
		ia.ia_mode = fat_make_mode(sbi, attr, S_IRWXUGO);
	else {
		ia.ia_mode = fat_make_mode(sbi, attr,
			S_IRUGO | S_IWUGO | (inode->i_mode & S_IXUGO));
	}

	/* The root directory has no attributes */
	if (inode->i_ino == MSDOS_ROOT_INO && attr != ATTR_DIR) {
		err = -EINVAL;
		goto out_unlock_inode;
	}

	if (sbi->options.sys_immutable &&
	    ((attr | oldattr) & ATTR_SYS) &&
	    !capable(CAP_LINUX_IMMUTABLE)) {
		err = -EPERM;
		goto out_unlock_inode;
	}

	/*
	 * The security check is questionable...  We single
	 * out the RO attribute for checking by the security
	 * module, just because it maps to a file mode.
	 */
	err = security_inode_setattr(file_mnt_idmap(file),
				     file->f_path.dentry, &ia);
	if (err)
		goto out_unlock_inode;

	/* This MUST be done before doing anything irreversible... */
	err = fat_setattr(file_mnt_idmap(file), file->f_path.dentry, &ia);
	if (err)
		goto out_unlock_inode;

	fsnotify_change(file->f_path.dentry, ia.ia_valid);
	if (sbi->options.sys_immutable) {
		if (attr & ATTR_SYS)
			inode->i_flags |= S_IMMUTABLE;
		else
			inode->i_flags &= ~S_IMMUTABLE;
	}

	fat_save_attrs(inode, attr);
	mark_inode_dirty(inode);
out_unlock_inode:
	inode_unlock(inode);
	mnt_drop_write_file(file);
out:
	return err;
}

static int fat_ioctl_get_volume_id(struct inode *inode, u32 __user *user_attr)
{
	struct msdos_sb_info *sbi = MSDOS_SB(inode->i_sb);
	return put_user(sbi->vol_id, user_attr);
}

static int fat_ioctl_fitrim(struct inode *inode, unsigned long arg)
{
	struct super_block *sb = inode->i_sb;
	struct fstrim_range __user *user_range;
	struct fstrim_range range;
	int err;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	if (!bdev_max_discard_sectors(sb->s_bdev))
		return -EOPNOTSUPP;

	user_range = (struct fstrim_range __user *)arg;
	if (copy_from_user(&range, user_range, sizeof(range)))
		return -EFAULT;

	range.minlen = max_t(unsigned int, range.minlen,
			     bdev_discard_granularity(sb->s_bdev));

	err = fat_trim_fs(inode, &range);
	if (err < 0)
		return err;

	if (copy_to_user(user_range, &range, sizeof(range)))
		return -EFAULT;

	return 0;
}

long fat_generic_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct inode *inode = file_inode(filp);
	u32 __user *user_attr = (u32 __user *)arg;

	switch (cmd) {
	case FAT_IOCTL_GET_ATTRIBUTES:
		return fat_ioctl_get_attributes(inode, user_attr);
	case FAT_IOCTL_SET_ATTRIBUTES:
		return fat_ioctl_set_attributes(filp, user_attr);
	case FAT_IOCTL_GET_VOLUME_ID:
		return fat_ioctl_get_volume_id(inode, user_attr);
	case FITRIM:
		return fat_ioctl_fitrim(inode, arg);
	default:
		return -ENOTTY;	/* Inappropriate ioctl for device */
	}
}

static int fat_file_release(struct inode *inode, struct file *filp)
{
	if ((filp->f_mode & FMODE_WRITE) &&
	    MSDOS_SB(inode->i_sb)->options.flush) {
		fat_flush_inodes(inode->i_sb, inode, NULL);
		set_current_state(TASK_UNINTERRUPTIBLE);
		io_schedule_timeout(HZ/10);
	}
	return 0;
}

int fat_file_fsync(struct file *filp, loff_t start, loff_t end, int datasync)
{
	struct inode *inode = filp->f_mapping->host;
	int err;

	err = __generic_file_fsync(filp, start, end, datasync);
	if (err)
		return err;

	err = sync_mapping_buffers(MSDOS_SB(inode->i_sb)->fat_inode->i_mapping);
	if (err)
		return err;

	return blkdev_issue_flush(inode->i_sb->s_bdev);
}


const struct file_operations fat_file_operations = {
	.llseek		= generic_file_llseek,
	.read_iter	= generic_file_read_iter,
	.write_iter	= generic_file_write_iter,
	.mmap		= generic_file_mmap,
	.release	= fat_file_release,
	.unlocked_ioctl	= fat_generic_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
	.fsync		= fat_file_fsync,
	.splice_read	= filemap_splice_read,
	.splice_write	= iter_file_splice_write,
	.fallocate	= fat_fallocate,
};

static int fat_cont_expand(struct inode *inode, loff_t size)
{
	struct address_space *mapping = inode->i_mapping;
	loff_t start = inode->i_size, count = size - inode->i_size;
	int err;

	err = generic_cont_expand_simple(inode, size);
	if (err)
		goto out;

	fat_truncate_time(inode, NULL, S_CTIME|S_MTIME);
	mark_inode_dirty(inode);
	if (IS_SYNC(inode)) {
		int err2;

		/*
		 * Opencode syncing since we don't have a file open to use
		 * standard fsync path.
		 */
		err = filemap_fdatawrite_range(mapping, start,
					       start + count - 1);
		err2 = sync_mapping_buffers(mapping);
		if (!err)
			err = err2;
		err2 = write_inode_now(inode, 1);
		if (!err)
			err = err2;
		if (!err) {
			err =  filemap_fdatawait_range(mapping, start,
						       start + count - 1);
		}
	}
out:
	return err;
}

/*
Linux fat fs预分配有两种方式
int fallocate(int fd, int mode, off_t offset, off_t len);
        1.参数mode带FALLOC_FL_KEEP_SIZE  : 分配磁盘空间、不会磁盘填充数据，速度快，文件size为0(后续写入时更新);
        2.参数mode不带FALLOC_FL_KEEP_SIZE: 分配磁盘空间、会磁盘填充数据，速度慢，文件size为传入的len;

产品采用了mode带FALLOC_FL_KEEP_SIZE的方式，且修改内核fat_fallocate接口、使得文件size为实际大小（应用需求）
对应的改动见下面的CONFIG_PRODUCT
*/

/*
 * Preallocate space for a file. This implements fat's fallocate file
 * operation, which gets called from sys_fallocate system call. User
 * space requests len bytes at offset. If FALLOC_FL_KEEP_SIZE is set
 * we just allocate clusters without zeroing them out. Otherwise we
 * allocate and zero out clusters via an expanding truncate.
 */
static long fat_fallocate(struct file *file, int mode,
			  loff_t offset, loff_t len)
{
	int nr_cluster; /* Number of clusters to be allocated */
	loff_t mm_bytes; /* Number of bytes to be allocated for file */
	loff_t ondisksize; /* block aligned on-disk size in bytes*/
	struct inode *inode = file->f_mapping->host;
	struct super_block *sb = inode->i_sb;
	struct msdos_sb_info *sbi = MSDOS_SB(sb);
	int err = 0;

	/* No support for hole punch or other fallocate flags. */
	if (mode & ~FALLOC_FL_KEEP_SIZE)
		return -EOPNOTSUPP;

	/* No support for dir */
	if (!S_ISREG(inode->i_mode))
		return -EOPNOTSUPP;

	inode_lock(inode);
	if (mode & FALLOC_FL_KEEP_SIZE) {
		ondisksize = inode->i_blocks << 9;
		if ((offset + len) <= ondisksize)
			goto error;

		/* First compute the number of clusters to be allocated */
		mm_bytes = offset + len - ondisksize;
		nr_cluster = (mm_bytes + (sbi->cluster_size - 1)) >>
			sbi->cluster_bits;

#ifndef CONFIG_PRODUCT
		/* Start the allocation.We are not zeroing out the clusters */
		while (nr_cluster-- > 0) {
			err = fat_add_cluster(inode);
			if (err)
				goto error;
		}
#else
		/* Start the allocation.We are not zeroing out the clusters */
		while (nr_cluster-- > 0) {
			err = fat_add_cluster(inode);
			if (err)
				goto error;
			/* update the allocated size */
			inode->i_size += sbi->cluster_size;
		}
		/* store the allocated size in the file entry of MS-DOS file system */
		MSDOS_I(inode)->mmu_private = inode->i_size;
		/* set inode dirty to make sure the lastest i_size will be updated to fat dentry */
		mark_inode_dirty(inode);
#endif
	} else {
		if ((offset + len) <= i_size_read(inode))
			goto error;

		/* This is just an expanding truncate */
		err = fat_cont_expand(inode, (offset + len));
	}

error:
	inode_unlock(inode);
	return err;
}

/*
 * 用于释放（回收）某个文件/目录从第skip个簇之后的所有簇，
 * 主要是在文件截断（如truncate）、删除文件时释放磁盘空间
 * 如果skip为0，则全部释放；否则保留前skip个簇
 */
/* Free all clusters after the skip'th cluster. */
static int fat_free(struct inode *inode, int skip)
{
	struct super_block *sb = inode->i_sb;
	int err, wait, free_start, i_start, i_logstart;

	/* 如果文件没有分配任何簇，直接返回 */
	if (MSDOS_I(inode)->i_start == 0)
		return 0;

	/* 无效inode的FAT缓存，保证后续操作的正确性 */
	fat_cache_inval_inode(inode);

	/* 获取是否需要回写磁盘 */
	wait = IS_DIRSYNC(inode);
	i_start = free_start = MSDOS_I(inode)->i_start;
	i_logstart = MSDOS_I(inode)->i_logstart;

	/* 如果是全部释放，则先更新起始簇号为0 */
	/* First, we write the new file size. */
	if (!skip) {
		MSDOS_I(inode)->i_start = 0;
		MSDOS_I(inode)->i_logstart = 0;
	}
	MSDOS_I(inode)->i_attrs |= ATTR_ARCH;
	fat_truncate_time(inode, NULL, S_CTIME|S_MTIME);
	if (wait) {
		/* 如果需要回写，则先将元数据写回磁盘 */
		err = fat_sync_inode(inode);
		if (err) {
			/* 写回失败则恢复原始起始簇号，防止内存和磁盘状态不一致 */
			MSDOS_I(inode)->i_start = i_start;
			MSDOS_I(inode)->i_logstart = i_logstart;
			return err;
		}
	} else
		/* 如果不用回写，否则只标记inode为脏，稍后释放后统一写回磁盘 */
		mark_inode_dirty(inode);

	/*
	 * 如果skip不为0，表示只释放部分簇链，需要在第skip-1个簇后写入新的EOF，
	 * 并获取后续需要释放的簇链起始簇号。
	 */
	/* Write a new EOF, and get the remaining cluster chain for freeing. */
	if (skip) {
		struct fat_entry fatent;
		int ret, fclus, dclus;

		/* 获取第skip-1个簇对应的FAT表项 */
		ret = fat_get_cluster(inode, skip - 1, &fclus, &dclus);
		if (ret < 0)
			return ret;
		else if (ret == FAT_ENT_EOF)	// 已经到达簇链末尾，无需释放
			return 0;

		/* 初始化FAT表项操作结构 */
		fatent_init(&fatent);
		/* 读取这个簇对应的FAT表项 */
		ret = fat_ent_read(inode, &fatent, dclus);
		if (ret == FAT_ENT_EOF) {	// 已经是EOF，无需处理
			fatent_brelse(&fatent);
			return 0;
		} else if (ret == FAT_ENT_FREE) { // 遇到空闲簇，说明簇链损坏
			/*
			 * 正常情况下，文件的簇链应该是连续的、有效的，
			 * 如果在遍历簇链是遇到空闲簇，则说明文件的簇链已经损坏了，可能的原因：
			 *	1.FAT表损坏；
			 *	2.链表断裂、簇被重新分配等；
			 *	3.文件系统bug;
			 */
			fat_fs_error(sb,
				     "%s: invalid cluster chain (i_pos %lld)",
				     __func__, MSDOS_I(inode)->i_pos);
			ret = -EIO;
		} else if (ret > 0) {
			/* 成功读取FAT表项后，将该簇的FAT表项写为EOF，截断簇链 */
			err = fat_ent_write(inode, &fatent, FAT_ENT_EOF, wait);
			if (err)
				ret = err;
		}
		/* 释放FAT表项操作结构 */
		fatent_brelse(&fatent);
		if (ret < 0)
			return ret;

		/* 需要释放的簇链起始簇号 */
		free_start = ret;
	}

	/* 更新文件大小，i_blocks，表示文件实际占用的块数 */
	inode->i_blocks = skip << (MSDOS_SB(sb)->cluster_bits - 9);

	/* 释放剩余的簇链（从free_start开始） */
	/* Freeing the remained cluster chain */
	return fat_free_clusters(inode, free_start);
}

void fat_truncate_blocks(struct inode *inode, loff_t offset)
{
	struct msdos_sb_info *sbi = MSDOS_SB(inode->i_sb);
	const unsigned int cluster_size = sbi->cluster_size;
	int nr_clusters;

	/*
	 * This protects against truncating a file bigger than it was then
	 * trying to write into the hole.
	 */
	if (MSDOS_I(inode)->mmu_private > offset)
		MSDOS_I(inode)->mmu_private = offset;

	nr_clusters = (offset + (cluster_size - 1)) >> sbi->cluster_bits;

	fat_free(inode, nr_clusters);
	fat_flush_inodes(inode->i_sb, inode, NULL);
}

int fat_getattr(struct mnt_idmap *idmap, const struct path *path,
		struct kstat *stat, u32 request_mask, unsigned int flags)
{
	struct inode *inode = d_inode(path->dentry);
	struct msdos_sb_info *sbi = MSDOS_SB(inode->i_sb);

	generic_fillattr(idmap, request_mask, inode, stat);
	stat->blksize = sbi->cluster_size;

	if (sbi->options.nfs == FAT_NFS_NOSTALE_RO) {
		/* Use i_pos for ino. This is used as fileid of nfs. */
		stat->ino = fat_i_pos_read(sbi, inode);
	}

	if (sbi->options.isvfat && request_mask & STATX_BTIME) {
		stat->result_mask |= STATX_BTIME;
		stat->btime = MSDOS_I(inode)->i_crtime;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(fat_getattr);

static int fat_sanitize_mode(const struct msdos_sb_info *sbi,
			     struct inode *inode, umode_t *mode_ptr)
{
	umode_t mask, perm;

	/*
	 * Note, the basic check is already done by a caller of
	 * (attr->ia_mode & ~FAT_VALID_MODE)
	 */

	if (S_ISREG(inode->i_mode))
		mask = sbi->options.fs_fmask;
	else
		mask = sbi->options.fs_dmask;

	perm = *mode_ptr & ~(S_IFMT | mask);

	/*
	 * Of the r and x bits, all (subject to umask) must be present. Of the
	 * w bits, either all (subject to umask) or none must be present.
	 *
	 * If fat_mode_can_hold_ro(inode) is false, can't change w bits.
	 */
	if ((perm & (S_IRUGO | S_IXUGO)) != (inode->i_mode & (S_IRUGO|S_IXUGO)))
		return -EPERM;
	if (fat_mode_can_hold_ro(inode)) {
		if ((perm & S_IWUGO) && ((perm & S_IWUGO) != (S_IWUGO & ~mask)))
			return -EPERM;
	} else {
		if ((perm & S_IWUGO) != (S_IWUGO & ~mask))
			return -EPERM;
	}

	*mode_ptr &= S_IFMT | perm;

	return 0;
}

static int fat_allow_set_time(struct mnt_idmap *idmap,
			      struct msdos_sb_info *sbi, struct inode *inode)
{
	umode_t allow_utime = sbi->options.allow_utime;

	if (!vfsuid_eq_kuid(i_uid_into_vfsuid(idmap, inode),
			    current_fsuid())) {
		if (vfsgid_in_group_p(i_gid_into_vfsgid(idmap, inode)))
			allow_utime >>= 3;
		if (allow_utime & MAY_WRITE)
			return 1;
	}

	/* use a default check */
	return 0;
}

#define TIMES_SET_FLAGS	(ATTR_MTIME_SET | ATTR_ATIME_SET | ATTR_TIMES_SET)
/* valid file mode bits */
#define FAT_VALID_MODE	(S_IFREG | S_IFDIR | S_IRWXUGO)

int fat_setattr(struct mnt_idmap *idmap, struct dentry *dentry,
		struct iattr *attr)
{
	struct msdos_sb_info *sbi = MSDOS_SB(dentry->d_sb);
	struct inode *inode = d_inode(dentry);
	unsigned int ia_valid;
	int error;

	/* Check for setting the inode time. */
	ia_valid = attr->ia_valid;
	if (ia_valid & TIMES_SET_FLAGS) {
		if (fat_allow_set_time(idmap, sbi, inode))
			attr->ia_valid &= ~TIMES_SET_FLAGS;
	}

	error = setattr_prepare(idmap, dentry, attr);
	attr->ia_valid = ia_valid;
	if (error) {
		if (sbi->options.quiet)
			error = 0;
		goto out;
	}

	/*
	 * Expand the file. Since inode_setattr() updates ->i_size
	 * before calling the ->truncate(), but FAT needs to fill the
	 * hole before it. XXX: this is no longer true with new truncate
	 * sequence.
	 */
	if (attr->ia_valid & ATTR_SIZE) {
		inode_dio_wait(inode);

		if (attr->ia_size > inode->i_size) {
			error = fat_cont_expand(inode, attr->ia_size);
			if (error || attr->ia_valid == ATTR_SIZE)
				goto out;
			attr->ia_valid &= ~ATTR_SIZE;
		}
	}

	if (((attr->ia_valid & ATTR_UID) &&
	     (!uid_eq(from_vfsuid(idmap, i_user_ns(inode), attr->ia_vfsuid),
		      sbi->options.fs_uid))) ||
	    ((attr->ia_valid & ATTR_GID) &&
	     (!gid_eq(from_vfsgid(idmap, i_user_ns(inode), attr->ia_vfsgid),
		      sbi->options.fs_gid))) ||
	    ((attr->ia_valid & ATTR_MODE) &&
	     (attr->ia_mode & ~FAT_VALID_MODE)))
		error = -EPERM;

	if (error) {
		if (sbi->options.quiet)
			error = 0;
		goto out;
	}

	/*
	 * We don't return -EPERM here. Yes, strange, but this is too
	 * old behavior.
	 */
	if (attr->ia_valid & ATTR_MODE) {
		if (fat_sanitize_mode(sbi, inode, &attr->ia_mode) < 0)
			attr->ia_valid &= ~ATTR_MODE;
	}

	if (attr->ia_valid & ATTR_SIZE) {
		error = fat_block_truncate_page(inode, attr->ia_size);
		if (error)
			goto out;
		down_write(&MSDOS_I(inode)->truncate_lock);
		truncate_setsize(inode, attr->ia_size);
		fat_truncate_blocks(inode, attr->ia_size);
		up_write(&MSDOS_I(inode)->truncate_lock);
	}

	/*
	 * setattr_copy can't truncate these appropriately, so we'll
	 * copy them ourselves
	 */
	if (attr->ia_valid & ATTR_ATIME)
		fat_truncate_time(inode, &attr->ia_atime, S_ATIME);
	if (attr->ia_valid & ATTR_CTIME)
		fat_truncate_time(inode, &attr->ia_ctime, S_CTIME);
	if (attr->ia_valid & ATTR_MTIME)
		fat_truncate_time(inode, &attr->ia_mtime, S_MTIME);
	attr->ia_valid &= ~(ATTR_ATIME|ATTR_CTIME|ATTR_MTIME);

	setattr_copy(idmap, inode, attr);
	mark_inode_dirty(inode);
out:
	return error;
}
EXPORT_SYMBOL_GPL(fat_setattr);

const struct inode_operations fat_file_inode_operations = {
	.setattr	= fat_setattr,
	.getattr	= fat_getattr,
	.update_time	= fat_update_time,
};
