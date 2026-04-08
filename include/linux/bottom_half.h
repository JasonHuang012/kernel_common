/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_BH_H
#define _LINUX_BH_H

#include <linux/instruction_pointer.h>
#include <linux/preempt.h>

#if defined(CONFIG_PREEMPT_RT) || defined(CONFIG_TRACE_IRQFLAGS)
extern void __local_bh_disable_ip(unsigned long ip, unsigned int cnt);
#else
static __always_inline void __local_bh_disable_ip(unsigned long ip, unsigned int cnt)
{
	/*
	 * 根据传入的cnt，往 preempt_count对应字段加上对应的值
	 * 比如传入SOFTIRQ_DISABLE_OFFSET, 表示既禁止抢占又禁止BH/softirq
	 */
	preempt_count_add(cnt);
	/* 编译屏障：防止编译器把临界区内的操作提前到 disable 之前 */
	barrier();
}
#endif

static inline void local_bh_disable(void)
{
	__local_bh_disable_ip(_THIS_IP_, SOFTIRQ_DISABLE_OFFSET);
}

extern void _local_bh_enable(void);
extern void __local_bh_enable_ip(unsigned long ip, unsigned int cnt);

static inline void local_bh_enable_ip(unsigned long ip)
{
	__local_bh_enable_ip(ip, SOFTIRQ_DISABLE_OFFSET);
}

static inline void local_bh_enable(void)
{
	__local_bh_enable_ip(_THIS_IP_, SOFTIRQ_DISABLE_OFFSET);
}

#ifdef CONFIG_PREEMPT_RT
extern bool local_bh_blocked(void);
#else
static inline bool local_bh_blocked(void) { return false; }
#endif

#endif /* _LINUX_BH_H */
