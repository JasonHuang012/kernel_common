/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Queue read/write lock
 *
 * These use generic atomic and locking routines, but depend on a fair spinlock
 * implementation in order to be fair themselves.  The implementation in
 * asm-generic/spinlock.h meets these requirements.
 *
 * (C) Copyright 2013-2014 Hewlett-Packard Development Company, L.P.
 *
 * Authors: Waiman Long <waiman.long@hp.com>
 */
#ifndef __ASM_GENERIC_QRWLOCK_H
#define __ASM_GENERIC_QRWLOCK_H

#include <linux/atomic.h>
#include <asm/barrier.h>
#include <asm/processor.h>

#include <asm-generic/qrwlock_types.h>

/* Must be included from asm/spinlock.h after defining arch_spin_is_locked.  */

/*
 * Writer states & reader shift and bias.
 */
#define	_QW_WAITING	0x100		/* A writer is waiting	   */
#define	_QW_LOCKED	0x0ff		/* A writer holds the lock */
#define	_QW_WMASK	0x1ff		/* Writer mask		   */
#define	_QR_SHIFT	9		/* Reader count shift	   */
#define _QR_BIAS	(1U << _QR_SHIFT)

/*
 * External function declarations
 */
extern void queued_read_lock_slowpath(struct qrwlock *lock);
extern void queued_write_lock_slowpath(struct qrwlock *lock);

/**
 * queued_read_trylock - try to acquire read lock of a queued rwlock
 * @lock : Pointer to queued rwlock structure
 * Return: 1 if lock acquired, 0 if failed
 */
static inline int queued_read_trylock(struct qrwlock *lock)
{
	int cnts;

	cnts = atomic_read(&lock->cnts);
	if (likely(!(cnts & _QW_WMASK))) {
		cnts = (u32)atomic_add_return_acquire(_QR_BIAS, &lock->cnts);
		if (likely(!(cnts & _QW_WMASK)))
			return 1;
		atomic_sub(_QR_BIAS, &lock->cnts);
	}
	return 0;
}

/**
 * queued_write_trylock - try to acquire write lock of a queued rwlock
 * @lock : Pointer to queued rwlock structure
 * Return: 1 if lock acquired, 0 if failed
 */
static inline int queued_write_trylock(struct qrwlock *lock)
{
	int cnts;

	cnts = atomic_read(&lock->cnts);
	if (unlikely(cnts))
		return 0;

	return likely(atomic_try_cmpxchg_acquire(&lock->cnts, &cnts,
				_QW_LOCKED));
}

/*
read_lock(lock)
    -->  _raw_read_lock()
        --> preempt_disable()
        --> do_raw_read_lock()
            --> arch_read_lock()
                --> queued_read_lock()
                    ↓（竞争时）
                    --> queued_read_lock_slowpath()
read_unlock(lock)
    -->  _raw_read_unlock()
        --> rwlock_release()
        --> do_raw_read_unlock()
            --> arch_read_unlock()
                --> queued_read_unlock()
	            --> (void)atomic_sub_return_release(_QR_BIAS, &lock->cnts);
        --> preempt_enable()
*
*/
/**
 * queued_read_lock - acquire read lock of a queued rwlock
 * @lock: Pointer to queued rwlock structure
 */
static inline void queued_read_lock(struct qrwlock *lock)
{
	int cnts;

        /*
         * acquire 语义：原子地将读者计数加 _QR_BIAS（0x200）
         * 同时读回新的 cnts 值
	 */
	cnts = atomic_add_return_acquire(_QR_BIAS, &lock->cnts);
        /*
         * Fast path：bits[8:0]（写者相关位）全为0，说明没有写者
         * 直接持锁成功，返回
	 */
	if (likely(!(cnts & _QW_WMASK)))
		return;

	/* The slowpath will decrement the reader count, if necessary. */
	/*
	 * Slow path：有写者存在（_QW_WAITING 或 _QW_LOCKED 置位）
	 * 进入队列等待，slowpath 内部会减回刚才加的 _QR_BIAS
	 */
	queued_read_lock_slowpath(lock);
}

/*
write_lock(lock)
    -->  _raw_write_lock()
        --> preempt_disable()
        --> do_raw_write_lock()
            --> arch_write_lock()
                --> queued_write_lock()
                    ↓（竞争时）
                    --> queued_write_lock_slowpath()
write_unlock(lock)
    -->  _raw_write_unlock()
        --> rwlock_release()
        --> do_raw_write_unlock()
            --> arch_write_unlock()
                --> queued_write_unlock()
	            --> smp_store_release(&lock->wlocked, 0);
        --> preempt_enable()
*/

/**
 * queued_write_lock - acquire write lock of a queued rwlock
 * @lock : Pointer to queued rwlock structure
 */
static inline void queued_write_lock(struct qrwlock *lock)
{
	int cnts = 0;
	/* Optimize for the unfair lock case where the fair flag is 0. */
	if (likely(atomic_try_cmpxchg_acquire(&lock->cnts, &cnts, _QW_LOCKED)))
		return;

	queued_write_lock_slowpath(lock);
}

/**
 * queued_read_unlock - release read lock of a queued rwlock
 * @lock : Pointer to queued rwlock structure
 */

/*
 * 读者解锁：原子减去 _QR_BIAS，release 语义
 * 当最后一个读者退出后，若有写者等待（cnts==_QW_WAITING），
 * 写者的 atomic_cond_read 循环会检测到并 CAS 成功
 */
static inline void queued_read_unlock(struct qrwlock *lock)
{
	/*
	 * Atomically decrement the reader count
	 */
	(void)atomic_sub_return_release(_QR_BIAS, &lock->cnts);
}

/**
 * queued_write_unlock - release write lock of a queued rwlock
 * @lock : Pointer to queued rwlock structure
 */

/*
 * 写者解锁：只清除 wlocked 字节（bits[7:0]），不改动其他位
 * 用 smp_store_release 而非 atomic：因为写者持锁期间不会有其他人修改 wlocked
 * release 语义：确保临界区写入对后续读者/写者可见
 */
static inline void queued_write_unlock(struct qrwlock *lock)
{
	smp_store_release(&lock->wlocked, 0);
	/*
	 * 写者解锁后：
	 * - 若有读者在等待（cnts 里有 _QR_BIAS 计数）：
	 *   它们在 slowpath 里 spin on _QW_LOCKED，现在清零 → 读者继续
	 * - 若有写者在 wait_lock 队列里：
	 *   写者拿到 wait_lock 后会检查 cnts，发现 0 → 直接 CAS 成功
	 */
}

/**
 * queued_rwlock_is_contended - check if the lock is contended
 * @lock : Pointer to queued rwlock structure
 * Return: 1 if lock contended, 0 otherwise
 */
static inline int queued_rwlock_is_contended(struct qrwlock *lock)
{
	return arch_spin_is_locked(&lock->wait_lock);
}

/*
 * Remapping rwlock architecture specific functions to the corresponding
 * queued rwlock functions.
 */
#define arch_read_lock(l)		queued_read_lock(l)
#define arch_write_lock(l)		queued_write_lock(l)
#define arch_read_trylock(l)		queued_read_trylock(l)
#define arch_write_trylock(l)		queued_write_trylock(l)
#define arch_read_unlock(l)		queued_read_unlock(l)
#define arch_write_unlock(l)		queued_write_unlock(l)
#define arch_rwlock_is_contended(l)	queued_rwlock_is_contended(l)

#endif /* __ASM_GENERIC_QRWLOCK_H */
