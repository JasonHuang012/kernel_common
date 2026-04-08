// SPDX-License-Identifier: GPL-2.0

/*
 * Generic wait-for-completion handler;
 *
 * It differs from semaphores in that their default case is the opposite,
 * wait_for_completion default blocks whereas semaphore default non-block. The
 * interface also makes it easy to 'complete' multiple waiting threads,
 * something which isn't entirely natural for semaphores.
 *
 * But more importantly, the primitive documents the usage. Semaphores would
 * typically be used for exclusion which gives rise to priority inversion.
 * Waiting for completion is a typically sync point, but not an exclusion point.
 */
/*
 * 函数作用：completion 的底层实现，递增 done 计数并唤醒一个 waiter
 * 参数：
 *   @x:          目标 completion
 *   @wake_flags: 传给 swake_up 的标志（如 WF_CURRENT_CPU）
 * 返回值：无
 */
static void complete_with_flags(struct completion *x, int wake_flags)
{
	unsigned long flags;

	/* 获取等待队列的自旋锁，同时关中断，防止中断上下文并发 */
	raw_spin_lock_irqsave(&x->wait.lock, flags);

	/* done 未达到 UINT_MAX（complete_all 标记）才递增，防止溢出回 0 */
	if (x->done != UINT_MAX)
		x->done++;
	/*
	 * 在持锁状态下唤醒(try_to_wake_up)队列中第一个 waiter（FIFO 顺序）,
	 * 等待队列链表的头部第一个成员
	 */
	swake_up_locked(&x->wait, wake_flags);
	/* 释放锁并恢复中断 */
	raw_spin_unlock_irqrestore(&x->wait.lock, flags);
}

void complete_on_current_cpu(struct completion *x)
{
	return complete_with_flags(x, WF_CURRENT_CPU);
}

/**
 * complete: - signals a single thread waiting on this completion
 * @x:  holds the state of this particular completion
 *
 * This will wake up a single thread waiting on this completion. Threads will be
 * awakened in the same order in which they were queued.
 *
 * See also complete_all(), wait_for_completion() and related routines.
 *
 * If this function wakes up a task, it executes a full memory barrier before
 * accessing the task state.
 */
void complete(struct completion *x)
{
	complete_with_flags(x, 0);
}
EXPORT_SYMBOL(complete);

/**
 * complete_all: - signals all threads waiting on this completion
 * @x:  holds the state of this particular completion
 *
 * This will wake up all threads waiting on this particular completion event.
 *
 * If this function wakes up a task, it executes a full memory barrier before
 * accessing the task state.
 *
 * Since complete_all() sets the completion of @x permanently to done
 * to allow multiple waiters to finish, a call to reinit_completion()
 * must be used on @x if @x is to be used again. The code must make
 * sure that all waiters have woken and finished before reinitializing
 * @x. Also note that the function completion_done() can not be used
 * to know if there are still waiters after complete_all() has been called.
 */
void complete_all(struct completion *x)
{
	unsigned long flags;

	lockdep_assert_RT_in_threaded_ctx();

	raw_spin_lock_irqsave(&x->wait.lock, flags);
	x->done = UINT_MAX;
	/* 循环唤醒swait等待队列链表的等待者，直到链表为空 */
	swake_up_all_locked(&x->wait);
	raw_spin_unlock_irqrestore(&x->wait.lock, flags);
}
EXPORT_SYMBOL(complete_all);

/*
 * 函数作用：completion 等待的核心实现，调用者持 wait.lock 进入
 * 参数：
 *   @x:       目标 completion
 *   @action:  调度函数指针，schedule_timeout 或 io_schedule_timeout
 *   @timeout: 超时 jiffies，MAX_SCHEDULE_TIMEOUT 表示永不超时
 *   @state:   任务等待状态（TASK_UNINTERRUPTIBLE / TASK_INTERRUPTIBLE / TASK_KILLABLE）
 * 返回值：剩余 jiffies（>0），0 表示超时，-ERESTARTSYS 表示被信号中断
 */
static inline long __sched
do_wait_for_common(struct completion *x,
		   long (*action)(long), long timeout, int state)
{
	/* 快速路径：done > 0 说明事件已发生，直接跳过等待 */
	if (!x->done) {
		/* 在栈上声明一个 swait 节点 */
		DECLARE_SWAITQUEUE(wait);

		do {
			/* 检查当前任务是否有待处理信号（仅 INTERRUPTIBLE/KILLABLE 状态有效） */
			if (signal_pending_state(state, current)) {
				timeout = -ERESTARTSYS;
				break;
			}
			/*
			 * 将当前任务加入等待队列（不设置 task state，下一行设置）
			  * 局部的wait加入到x->wait中
			 */
			__prepare_to_swait(&x->wait, &wait);
			/* 设置当前任务状态为等待态，调度器不会再调度它 */
			__set_current_state(state);
			/* 释放锁后才能调度出去，避免持锁睡眠 */
			raw_spin_unlock_irq(&x->wait.lock);
			/* 调用 schedule_timeout 或 io_schedule_timeout，让出 CPU */
			timeout = action(timeout);
			/* 被唤醒后重新获取锁，检查 done 状态 */
			raw_spin_lock_irq(&x->wait.lock);
		/* done 仍为 0 且未超时则继续等待 */
		} while (!x->done && timeout);

		/* done不为0，或者超时了，从等待队列中移除自己 */
		__finish_swait(&x->wait, &wait);
		/* 超时或被信号中断退出循环，done 仍为 0，直接返回 timeout（0 或负值） */
		if (!x->done)
			return timeout;
	}
	/* done 为 UINT_MAX（complete_all）时不递减，否则消费一次完成事件 */
	if (x->done != UINT_MAX)
		x->done--;

      /* 返回剩余 jiffies，至少为 1（区分"完成"和"超时返回 0"） */
	return timeout ?: 1;
}

static inline long __sched
__wait_for_common(struct completion *x,
		  long (*action)(long), long timeout, int state)
{
	might_sleep();

	complete_acquire(x);

	/*
	 * 获取swait的自旋锁
	 * 因为后面需要将task加入swait的等待队列中，所以需要先拿锁
	 * 进入等待队列、睡眠啊之前会先unlock，免得死锁unlock
	 */
	raw_spin_lock_irq(&x->wait.lock);
	/* 将task加入swait 等待队列中，等待被唤醒, complete完成，则返回 */
	timeout = do_wait_for_common(x, action, timeout, state);
	/* complete完成后，会将当前task从swait等待队列移除，会拿锁，所以这里要unlock */
	raw_spin_unlock_irq(&x->wait.lock);

	complete_release(x);

	return timeout;
}

static long __sched
wait_for_common(struct completion *x, long timeout, int state)
{
	return __wait_for_common(x, schedule_timeout, timeout, state);
}

static long __sched
wait_for_common_io(struct completion *x, long timeout, int state)
{
	return __wait_for_common(x, io_schedule_timeout, timeout, state);
}

/**
 * wait_for_completion: - waits for completion of a task
 * @x:  holds the state of this particular completion
 *
 * This waits to be signaled for completion of a specific task. It is NOT
 * interruptible and there is no timeout.
 *
 * See also similar routines (i.e. wait_for_completion_timeout()) with timeout
 * and interrupt capability. Also see complete().
 */
void __sched wait_for_completion(struct completion *x)
{
	wait_for_common(x, MAX_SCHEDULE_TIMEOUT, TASK_UNINTERRUPTIBLE);
}
EXPORT_SYMBOL(wait_for_completion);

/**
 * wait_for_completion_timeout: - waits for completion of a task (w/timeout)
 * @x:  holds the state of this particular completion
 * @timeout:  timeout value in jiffies
 *
 * This waits for either a completion of a specific task to be signaled or for a
 * specified timeout to expire. The timeout is in jiffies. It is not
 * interruptible.
 *
 * Return: 0 if timed out, and positive (at least 1, or number of jiffies left
 * till timeout) if completed.
 */
unsigned long __sched
wait_for_completion_timeout(struct completion *x, unsigned long timeout)
{
	return wait_for_common(x, timeout, TASK_UNINTERRUPTIBLE);
}
EXPORT_SYMBOL(wait_for_completion_timeout);

/**
 * wait_for_completion_io: - waits for completion of a task
 * @x:  holds the state of this particular completion
 *
 * This waits to be signaled for completion of a specific task. It is NOT
 * interruptible and there is no timeout. The caller is accounted as waiting
 * for IO (which traditionally means blkio only).
 */
void __sched wait_for_completion_io(struct completion *x)
{
	wait_for_common_io(x, MAX_SCHEDULE_TIMEOUT, TASK_UNINTERRUPTIBLE);
}
EXPORT_SYMBOL(wait_for_completion_io);

/**
 * wait_for_completion_io_timeout: - waits for completion of a task (w/timeout)
 * @x:  holds the state of this particular completion
 * @timeout:  timeout value in jiffies
 *
 * This waits for either a completion of a specific task to be signaled or for a
 * specified timeout to expire. The timeout is in jiffies. It is not
 * interruptible. The caller is accounted as waiting for IO (which traditionally
 * means blkio only).
 *
 * Return: 0 if timed out, and positive (at least 1, or number of jiffies left
 * till timeout) if completed.
 */
unsigned long __sched
wait_for_completion_io_timeout(struct completion *x, unsigned long timeout)
{
	return wait_for_common_io(x, timeout, TASK_UNINTERRUPTIBLE);
}
EXPORT_SYMBOL(wait_for_completion_io_timeout);

/**
 * wait_for_completion_interruptible: - waits for completion of a task (w/intr)
 * @x:  holds the state of this particular completion
 *
 * This waits for completion of a specific task to be signaled. It is
 * interruptible.
 *
 * Return: -ERESTARTSYS if interrupted, 0 if completed.
 */
int __sched wait_for_completion_interruptible(struct completion *x)
{
	long t = wait_for_common(x, MAX_SCHEDULE_TIMEOUT, TASK_INTERRUPTIBLE);

	if (t == -ERESTARTSYS)
		return t;
	return 0;
}
EXPORT_SYMBOL(wait_for_completion_interruptible);

/**
 * wait_for_completion_interruptible_timeout: - waits for completion (w/(to,intr))
 * @x:  holds the state of this particular completion
 * @timeout:  timeout value in jiffies
 *
 * This waits for either a completion of a specific task to be signaled or for a
 * specified timeout to expire. It is interruptible. The timeout is in jiffies.
 *
 * Return: -ERESTARTSYS if interrupted, 0 if timed out, positive (at least 1,
 * or number of jiffies left till timeout) if completed.
 */
long __sched
wait_for_completion_interruptible_timeout(struct completion *x,
					  unsigned long timeout)
{
	return wait_for_common(x, timeout, TASK_INTERRUPTIBLE);
}
EXPORT_SYMBOL(wait_for_completion_interruptible_timeout);

/**
 * wait_for_completion_killable: - waits for completion of a task (killable)
 * @x:  holds the state of this particular completion
 *
 * This waits to be signaled for completion of a specific task. It can be
 * interrupted by a kill signal.
 *
 * Return: -ERESTARTSYS if interrupted, 0 if completed.
 */
int __sched wait_for_completion_killable(struct completion *x)
{
	long t = wait_for_common(x, MAX_SCHEDULE_TIMEOUT, TASK_KILLABLE);

	if (t == -ERESTARTSYS)
		return t;
	return 0;
}
EXPORT_SYMBOL(wait_for_completion_killable);

int __sched wait_for_completion_state(struct completion *x, unsigned int state)
{
	long t = wait_for_common(x, MAX_SCHEDULE_TIMEOUT, state);

	if (t == -ERESTARTSYS)
		return t;
	return 0;
}
EXPORT_SYMBOL(wait_for_completion_state);

/**
 * wait_for_completion_killable_timeout: - waits for completion of a task (w/(to,killable))
 * @x:  holds the state of this particular completion
 * @timeout:  timeout value in jiffies
 *
 * This waits for either a completion of a specific task to be
 * signaled or for a specified timeout to expire. It can be
 * interrupted by a kill signal. The timeout is in jiffies.
 *
 * Return: -ERESTARTSYS if interrupted, 0 if timed out, positive (at least 1,
 * or number of jiffies left till timeout) if completed.
 */
long __sched
wait_for_completion_killable_timeout(struct completion *x,
				     unsigned long timeout)
{
	return wait_for_common(x, timeout, TASK_KILLABLE);
}
EXPORT_SYMBOL(wait_for_completion_killable_timeout);

/**
 *	try_wait_for_completion - try to decrement a completion without blocking
 *	@x:	completion structure
 *
 *	Return: 0 if a decrement cannot be done without blocking
 *		 1 if a decrement succeeded.
 *
 *	If a completion is being used as a counting completion,
 *	attempt to decrement the counter without blocking. This
 *	enables us to avoid waiting if the resource the completion
 *	is protecting is not available.
 */
/* 非阻塞尝试 */
bool try_wait_for_completion(struct completion *x)
{
	unsigned long flags;
	bool ret = true;

	/*
	 * Since x->done will need to be locked only
	 * in the non-blocking case, we check x->done
	 * first without taking the lock so we can
	 * return early in the blocking case.
	 */
	if (!READ_ONCE(x->done))
		return false;

	raw_spin_lock_irqsave(&x->wait.lock, flags);
	if (!x->done)
		ret = false;
	else if (x->done != UINT_MAX)
		x->done--;
	raw_spin_unlock_irqrestore(&x->wait.lock, flags);
	return ret;
}
EXPORT_SYMBOL(try_wait_for_completion);

/**
 *	completion_done - Test to see if a completion has any waiters
 *	@x:	completion structure
 *
 *	Return: 0 if there are waiters (wait_for_completion() in progress)
 *		 1 if there are no waiters.
 *
 *	Note, this will always return true if complete_all() was called on @X.
 */
bool completion_done(struct completion *x)
{
	unsigned long flags;

	if (!READ_ONCE(x->done))
		return false;

	/*
	 * If ->done, we need to wait for complete() to release ->wait.lock
	 * otherwise we can end up freeing the completion before complete()
	 * is done referencing it.
	 */
	raw_spin_lock_irqsave(&x->wait.lock, flags);
	raw_spin_unlock_irqrestore(&x->wait.lock, flags);
	return true;
}
EXPORT_SYMBOL(completion_done);
