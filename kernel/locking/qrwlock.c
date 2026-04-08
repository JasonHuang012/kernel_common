// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Queued read/write locks
 *
 * (C) Copyright 2013-2014 Hewlett-Packard Development Company, L.P.
 *
 * Authors: Waiman Long <waiman.long@hp.com>
 */
#include <linux/smp.h>
#include <linux/bug.h>
#include <linux/cpumask.h>
#include <linux/percpu.h>
#include <linux/hardirq.h>
#include <linux/spinlock.h>
#include <trace/events/lock.h>

/**
 * queued_read_lock_slowpath - acquire read lock of a queued rwlock
 * @lock: Pointer to queued rwlock structure
 */
void __lockfunc queued_read_lock_slowpath(struct qrwlock *lock)
{
	/*
	 * Readers come here when they cannot get the lock without waiting
	 */
	if (unlikely(in_interrupt())) {
		/*
		 * Readers in interrupt context will get the lock immediately
		 * if the writer is just waiting (not holding the lock yet),
		 * so spin with ACQUIRE semantics until the lock is available
		 * without waiting in the queue.
		 */
		/*
		 * 中断上下文中不能睡眠，也不能拿 wait_lock（可能死锁）
		 * 特殊处理：只等 _QW_LOCKED 清零，不等 _QW_WAITING
		 * 即：写者"已持锁"时等，写者"仅等待"时不等
		 * 这是为了防止中断与写者死锁：
		 *   写者持 wait_lock 等读者退出 → 中断来了也等写者 → 死锁
		 */
		atomic_cond_read_acquire(&lock->cnts, !(VAL & _QW_LOCKED));
		return;
	}
	/*
	 * 进程上下文：先把之前加的 _QR_BIAS 减回去
	 * （快速路径里加了，但有写者，所以先退出来）
	 */
	atomic_sub(_QR_BIAS, &lock->cnts);

	trace_contention_begin(lock, LCB_F_SPIN | LCB_F_READ);

	/*
	 * Put the reader into the wait queue
	 */
	/*
	 * 排入等待队列：拿 wait_lock（这里会自旋等待，体现队列特性）
	 * 多个等待的读者在这里串行化，避免 cache 颠簸
	 */
	arch_spin_lock(&lock->wait_lock);
	/* 重新加回 _QR_BIAS，正式表示"我是一个等待中的读者" */
	atomic_add(_QR_BIAS, &lock->cnts);

	/*
	 * The ACQUIRE semantics of the following spinning code ensure
	 * that accesses can't leak upwards out of our subsequent critical
	 * section in the case that the lock is currently held for write.
	 */
	/* 重新加回 _QR_BIAS，正式表示"我是一个等待中的读者" */
	atomic_cond_read_acquire(&lock->cnts, !(VAL & _QW_LOCKED));

	/*
	 * Signal the next one in queue to become queue head
	 */
	/* 拿到锁，从 wait_lock 队列退出，唤醒下一个等待者 */
	arch_spin_unlock(&lock->wait_lock);

	trace_contention_end(lock, 0);
}
EXPORT_SYMBOL(queued_read_lock_slowpath);

/**
 * queued_write_lock_slowpath - acquire write lock of a queued rwlock
 * @lock : Pointer to queued rwlock structure
 */
void __lockfunc queued_write_lock_slowpath(struct qrwlock *lock)
{
	int cnts;

	trace_contention_begin(lock, LCB_F_SPIN | LCB_F_WRITE);

	/* Put the writer into the wait queue */

        /* 进入写者等待队列：拿 wait_lock，序列化多个写者
         * 只有一个写者能继续往下走，其它写者在这里自旋等待
	 */
	arch_spin_lock(&lock->wait_lock);

	/* Try to acquire the lock directly if no reader is present */
	/* 再次尝试直接获取：如果此时恰好没有读者也没有写者 */
	if (!(cnts = atomic_read(&lock->cnts)) &&
	    atomic_try_cmpxchg_acquire(&lock->cnts, &cnts, _QW_LOCKED))
		goto unlock;

	/* Set the waiting flag to notify readers that a writer is pending */
	/*
	 * ★ 防饥饿的关键操作 ★
	 * 设置 _QW_WAITING 标志（bit8），通知所有新来的读者：
	 * "有写者在等了，你们要去 wait_lock 排队，不能直接加读锁"
	 *
	 * 之后新来的读者调用 queued_read_lock() 时：
	 *   cnts & _QW_WMASK != 0（因为 _QW_WAITING 位已置）
	 *   → 进入 slowpath → arch_spin_lock(wait_lock) 排到写者后面
	 * 这样写者不会被源源不断的新读者饿死
	 */
	atomic_or(_QW_WAITING, &lock->cnts);

	/* When no more readers or writers, set the locked flag */
	/*
	 * 自旋等待：直到 cnts 变为 _QW_WAITING（即读者全部退出）
	 * 条件：cnts 恰好等于 _QW_WAITING，说明只剩写等待标志，无读者
	 */
	do {
		cnts = atomic_cond_read_relaxed(&lock->cnts, VAL == _QW_WAITING);
	} while (!atomic_try_cmpxchg_acquire(&lock->cnts, &cnts, _QW_LOCKED));
unlock:
	arch_spin_unlock(&lock->wait_lock);

	trace_contention_end(lock, 0);
}
EXPORT_SYMBOL(queued_write_lock_slowpath);
