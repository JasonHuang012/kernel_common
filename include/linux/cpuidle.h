/*
 * cpuidle.h - a generic framework for CPU idle power management
 *
 * (C) 2007 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>
 *          Shaohua Li <shaohua.li@intel.com>
 *          Adam Belay <abelay@novell.com>
 *
 * This code is licenced under the GPL.
 */

#ifndef _LINUX_CPUIDLE_H
#define _LINUX_CPUIDLE_H

#include <linux/percpu.h>
#include <linux/list.h>
#include <linux/hrtimer.h>
#include <linux/context_tracking.h>

#define CPUIDLE_STATE_MAX	10
#define CPUIDLE_NAME_LEN	16
#define CPUIDLE_DESC_LEN	32

struct module;

struct cpuidle_device;
struct cpuidle_driver;


/****************************
 * CPUIDLE DEVICE INTERFACE *
 ****************************/

#define CPUIDLE_STATE_DISABLED_BY_USER		BIT(0)
#define CPUIDLE_STATE_DISABLED_BY_DRIVER	BIT(1)

/*
 * cpuidle的状态信息记录，一般会有几种cpu空闲状态，state0\state1等
 * 节点路径：/sys/devices/system/cpu/cpu1/cpuidle/state0/
 */
struct cpuidle_state_usage {
	unsigned long long	disable;	// 是否禁用该状态
	/* 进入该状态的次数，用于分析cpu空闲状态的使用频率 */
	unsigned long long	usage;
	/* 进入该状态的累计时间，用于计算平均驻留时间和功耗调优 */
	u64			time_ns;
	/*
	 * too deep次数统计, 实际驻留次数 > 目标驻留次数
	 * 当cpu实际空闲时间超过target_residency太多，表明这个状态too deep
	 * 表示退出延迟开销不合理？
	 */
	unsigned long long	above; /* Number of times it's been too deep */
	/*
	 * too low次数统计，实际驻留次数 < 目标驻留次数
	 * 当cpu实际空闲时间小于target_residency
	 * 表示过早被唤醒？
	 */
	unsigned long long	below; /* Number of times it's been too shallow */
	/*
	 * 请求进入这个状态而被拒绝的次数
	 * 可能是空闲时间不足、硬件限制等
	 */
	unsigned long long	rejected; /* Number of times idle entry was rejected */
#ifdef CONFIG_SUSPEND
	/*
	 * s2idle： 系统级浅睡眠状态，比普通的空闲状态更深，但是比重启快
	 *
	 * s2idle_usage: s2idle次数
	 * s2idle_time: s2idle时间
	 */
	unsigned long long	s2idle_usage;
	unsigned long long	s2idle_time; /* in US */
#endif
};

struct cpuidle_state {
	/*
	 * cpuidle state的名字
	 * /sys/devices/system/cpu/cpu0/cpuidle# cat state0/name
	 * WFI
	 * /sys/devices/system/cpu/cpu0/cpuidle# cat state1/name
	 * cpu-sleep-0
	 */
	char		name[CPUIDLE_NAME_LEN];
	/*
	 * cpuidle state的描述
	 * /sys/devices/system/cpu/cpu0/cpuidle# cat state0/desc
	 * ARM WFI
	 * /sys/devices/system/cpu/cpu0/cpuidle# cat state1/desc
	 * cpu-sleep-0
	 */
	char		desc[CPUIDLE_DESC_LEN];

	/*
	 * 退出延迟
	 * 从该状态唤醒到运行状态所需的时间
	 * 延迟越低、响应越快
	 */
	s64		exit_latency_ns;
	/*
	 * 目标驻留时间
	 * 为了补偿退出延迟，cpu需要在这个状态至少停留的时间
	 * 如果空闲时间 < target_residency_ns，选择更浅的状态更高效
	 * 节省的功耗 > 退出延迟的功耗开销
	 */
	s64		target_residency_ns;
	/* 状态标志，下面的CPUIDLE_FLAG_* */
	unsigned int	flags;
	unsigned int	exit_latency; /* in US */
	/* 功耗，不同状态下功耗不同 */
	int		power_usage; /* in mW */
	unsigned int	target_residency; /* in US */

	/*
	 * 进入状态的函数
	 * 进入该状态的操作，比如硬件特定的操作：MWAIT指令、关闭时钟、降低电压等
	 */
	int (*enter)	(struct cpuidle_device *dev,
			struct cpuidle_driver *drv,
			int index);

	/*
	 * 进入深度休眠
	 * 用于cpu热插拔、当cpu下线时进入的状态
	 */
	int (*enter_dead) (struct cpuidle_device *dev, int index);

	/*
	 * CPUs execute ->enter_s2idle with the local tick or entire timekeeping
	 * suspended, so it must not re-enable interrupts at any point (even
	 * temporarily) or attempt to change states of clock event devices.
	 *
	 * This callback may point to the same function as ->enter if all of
	 * the above requirements are met by it.
	 */
	/*
	 * 进入i2idle(系统挂起)
	 */
	int (*enter_s2idle)(struct cpuidle_device *dev,
			    struct cpuidle_driver *drv,
			    int index);
};

/*
# 查看CPU0的所有空闲状态
$ ls /sys/devices/system/cpu/cpu0/cpuidle/
state0/ state1/ state2/ state3/ state4/

# 查看C1状态的信息
$ cat /sys/devices/system/cpu/cpu0/cpuidle/state1/name
C1

$ cat /sys/devices/system/cpu/cpu0/cpuidle/state1/desc
MWAIT 0x00

$ cat /sys/devices/system/cpu/cpu0/cpuidle/state1/latency
1          # 1微秒退出延迟

$ cat /sys/devices/system/cpu/cpu0/cpuidle/state1/residency
2          # 目标驻留2微秒

$ cat /sys/devices/system/cpu/cpu0/cpuidle/state1/power
10         # 功耗10mW

# 查看统计信息
$ cat /sys/devices/system/cpu/cpu0/cpuidle/state1/usage
125432     # 进入次数

$ cat /sys/devices/system/cpu/cpu0/cpuidle/state1/time
1234567890 # 总停留时间（纳秒）


调优示例
# 1. 禁用某个状态（如果发现它效率低）
echo 1 > /sys/devices/system/cpu/cpu0/cpuidle/state2/disable

# 2. 查看调优数据
$ cat /sys/devices/system/cpu/cpu0/cpuidle/state2/below
1523       # 1523次提前唤醒 → 这个状态可能太深

$ cat /sys/devices/system/cpu/cpu0/cpuidle/state2/above
23         # 只有23次过度休眠 → 状态选择还算合理
*/

/* Idle State Flags */
#define CPUIDLE_FLAG_NONE       	(0x00)
#define CPUIDLE_FLAG_POLLING		BIT(0) /* polling state */
#define CPUIDLE_FLAG_COUPLED		BIT(1) /* state applies to multiple cpus */
#define CPUIDLE_FLAG_TIMER_STOP 	BIT(2) /* timer is stopped on this state */
#define CPUIDLE_FLAG_UNUSABLE		BIT(3) /* avoid using this state */
#define CPUIDLE_FLAG_OFF		BIT(4) /* disable this state by default */
#define CPUIDLE_FLAG_TLB_FLUSHED	BIT(5) /* idle-state flushes TLBs */
#define CPUIDLE_FLAG_RCU_IDLE		BIT(6) /* idle-state takes care of RCU */

struct cpuidle_device_kobj;
struct cpuidle_state_kobj;
struct cpuidle_driver_kobj;

struct cpuidle_device {
	unsigned int		registered:1;
	unsigned int		enabled:1;
	unsigned int		poll_time_limit:1;
	unsigned int		cpu;
	ktime_t			next_hrtimer;

	int			last_state_idx;
	u64			last_residency_ns;
	u64			poll_limit_ns;
	u64			forced_idle_latency_limit_ns;
	struct cpuidle_state_usage	states_usage[CPUIDLE_STATE_MAX];
	struct cpuidle_state_kobj *kobjs[CPUIDLE_STATE_MAX];
	struct cpuidle_driver_kobj *kobj_driver;
	struct cpuidle_device_kobj *kobj_dev;
	struct list_head 	device_list;

#ifdef CONFIG_ARCH_NEEDS_CPU_IDLE_COUPLED
	cpumask_t		coupled_cpus;
	struct cpuidle_coupled	*coupled;
#endif
};

DECLARE_PER_CPU(struct cpuidle_device *, cpuidle_devices);
DECLARE_PER_CPU(struct cpuidle_device, cpuidle_dev);

static __always_inline void ct_cpuidle_enter(void)
{
	lockdep_assert_irqs_disabled();
	/*
	 * Idle is allowed to (temporary) enable IRQs. It
	 * will return with IRQs disabled.
	 *
	 * Trace IRQs enable here, then switch off RCU, and have
	 * arch_cpu_idle() use raw_local_irq_enable(). Note that
	 * ct_idle_enter() relies on lockdep IRQ state, so switch that
	 * last -- this is very similar to the entry code.
	 */
	trace_hardirqs_on_prepare();
	lockdep_hardirqs_on_prepare();
	instrumentation_end();
	ct_idle_enter();
	lockdep_hardirqs_on(_RET_IP_);
}

static __always_inline void ct_cpuidle_exit(void)
{
	/*
	 * Carefully undo the above.
	 */
	lockdep_hardirqs_off(_RET_IP_);
	ct_idle_exit();
	instrumentation_begin();
}

/****************************
 * CPUIDLE DRIVER INTERFACE *
 ****************************/

struct cpuidle_driver {
	const char		*name;
	struct module 		*owner;

        /* used by the cpuidle framework to setup the broadcast timer */
	unsigned int            bctimer:1;
	/* states array must be ordered in decreasing power consumption */
	struct cpuidle_state	states[CPUIDLE_STATE_MAX];
	int			state_count;
	int			safe_state_index;

	/* the driver handles the cpus in cpumask */
	struct cpumask		*cpumask;

	/* preferred governor to switch at register time */
	const char		*governor;
};

#ifdef CONFIG_CPU_IDLE
extern void disable_cpuidle(void);
extern bool cpuidle_not_available(struct cpuidle_driver *drv,
				  struct cpuidle_device *dev);

extern int cpuidle_select(struct cpuidle_driver *drv,
			  struct cpuidle_device *dev,
			  bool *stop_tick);
extern int cpuidle_enter(struct cpuidle_driver *drv,
			 struct cpuidle_device *dev, int index);
extern void cpuidle_reflect(struct cpuidle_device *dev, int index);
extern u64 cpuidle_poll_time(struct cpuidle_driver *drv,
			     struct cpuidle_device *dev);

extern int cpuidle_register_driver(struct cpuidle_driver *drv);
extern struct cpuidle_driver *cpuidle_get_driver(void);
extern void cpuidle_driver_state_disabled(struct cpuidle_driver *drv, int idx,
					bool disable);
extern void cpuidle_unregister_driver(struct cpuidle_driver *drv);
extern int cpuidle_register_device(struct cpuidle_device *dev);
extern void cpuidle_unregister_device(struct cpuidle_device *dev);
extern int cpuidle_register(struct cpuidle_driver *drv,
			    const struct cpumask *const coupled_cpus);
extern void cpuidle_unregister(struct cpuidle_driver *drv);
extern void cpuidle_pause_and_lock(void);
extern void cpuidle_resume_and_unlock(void);
extern void cpuidle_pause(void);
extern void cpuidle_resume(void);
extern int cpuidle_enable_device(struct cpuidle_device *dev);
extern void cpuidle_disable_device(struct cpuidle_device *dev);
extern int cpuidle_play_dead(void);

extern struct cpuidle_driver *cpuidle_get_cpu_driver(struct cpuidle_device *dev);
static inline struct cpuidle_device *cpuidle_get_device(void)
{return __this_cpu_read(cpuidle_devices); }
#else
static inline void disable_cpuidle(void) { }
static inline bool cpuidle_not_available(struct cpuidle_driver *drv,
					 struct cpuidle_device *dev)
{return true; }
static inline int cpuidle_select(struct cpuidle_driver *drv,
				 struct cpuidle_device *dev, bool *stop_tick)
{return -ENODEV; }
static inline int cpuidle_enter(struct cpuidle_driver *drv,
				struct cpuidle_device *dev, int index)
{return -ENODEV; }
static inline void cpuidle_reflect(struct cpuidle_device *dev, int index) { }
static inline u64 cpuidle_poll_time(struct cpuidle_driver *drv,
			     struct cpuidle_device *dev)
{return 0; }
static inline int cpuidle_register_driver(struct cpuidle_driver *drv)
{return -ENODEV; }
static inline struct cpuidle_driver *cpuidle_get_driver(void) {return NULL; }
static inline void cpuidle_driver_state_disabled(struct cpuidle_driver *drv,
					       int idx, bool disable) { }
static inline void cpuidle_unregister_driver(struct cpuidle_driver *drv) { }
static inline int cpuidle_register_device(struct cpuidle_device *dev)
{return -ENODEV; }
static inline void cpuidle_unregister_device(struct cpuidle_device *dev) { }
static inline int cpuidle_register(struct cpuidle_driver *drv,
				   const struct cpumask *const coupled_cpus)
{return -ENODEV; }
static inline void cpuidle_unregister(struct cpuidle_driver *drv) { }
static inline void cpuidle_pause_and_lock(void) { }
static inline void cpuidle_resume_and_unlock(void) { }
static inline void cpuidle_pause(void) { }
static inline void cpuidle_resume(void) { }
static inline int cpuidle_enable_device(struct cpuidle_device *dev)
{return -ENODEV; }
static inline void cpuidle_disable_device(struct cpuidle_device *dev) { }
static inline int cpuidle_play_dead(void) {return -ENODEV; }
static inline struct cpuidle_driver *cpuidle_get_cpu_driver(
	struct cpuidle_device *dev) {return NULL; }
static inline struct cpuidle_device *cpuidle_get_device(void) {return NULL; }
#endif

#ifdef CONFIG_CPU_IDLE
extern int cpuidle_find_deepest_state(struct cpuidle_driver *drv,
				      struct cpuidle_device *dev,
				      u64 latency_limit_ns);
extern int cpuidle_enter_s2idle(struct cpuidle_driver *drv,
				struct cpuidle_device *dev);
extern void cpuidle_use_deepest_state(u64 latency_limit_ns);
#else
static inline int cpuidle_find_deepest_state(struct cpuidle_driver *drv,
					     struct cpuidle_device *dev,
					     u64 latency_limit_ns)
{return -ENODEV; }
static inline int cpuidle_enter_s2idle(struct cpuidle_driver *drv,
				       struct cpuidle_device *dev)
{return -ENODEV; }
static inline void cpuidle_use_deepest_state(u64 latency_limit_ns)
{
}
#endif

/* kernel/sched/idle.c */
extern void sched_idle_set_state(struct cpuidle_state *idle_state);
extern void default_idle_call(void);

#ifdef CONFIG_ARCH_NEEDS_CPU_IDLE_COUPLED
void cpuidle_coupled_parallel_barrier(struct cpuidle_device *dev, atomic_t *a);
#else
static inline void cpuidle_coupled_parallel_barrier(struct cpuidle_device *dev, atomic_t *a)
{
}
#endif

#if defined(CONFIG_CPU_IDLE) && defined(CONFIG_ARCH_HAS_CPU_RELAX)
void cpuidle_poll_state_init(struct cpuidle_driver *drv);
#else
static inline void cpuidle_poll_state_init(struct cpuidle_driver *drv) {}
#endif

/******************************
 * CPUIDLE GOVERNOR INTERFACE *
 ******************************/

struct cpuidle_governor {
	char			name[CPUIDLE_NAME_LEN];
	struct list_head 	governor_list;
	unsigned int		rating;

	int  (*enable)		(struct cpuidle_driver *drv,
					struct cpuidle_device *dev);
	void (*disable)		(struct cpuidle_driver *drv,
					struct cpuidle_device *dev);

	int  (*select)		(struct cpuidle_driver *drv,
					struct cpuidle_device *dev,
					bool *stop_tick);
	void (*reflect)		(struct cpuidle_device *dev, int index);
};

extern int cpuidle_register_governor(struct cpuidle_governor *gov);
extern s64 cpuidle_governor_latency_req(unsigned int cpu);

#define __CPU_PM_CPU_IDLE_ENTER(low_level_idle_enter,			\
				idx,					\
				state,					\
				is_retention, is_rcu)			\
({									\
	int __ret = 0;							\
									\
	if (!idx) {							\
		cpu_do_idle();						\
		return idx;						\
	}								\
									\
	if (!is_retention)						\
		__ret =  cpu_pm_enter();				\
	if (!__ret) {							\
		if (!is_rcu)						\
			ct_cpuidle_enter();				\
		__ret = low_level_idle_enter(state);			\
		if (!is_rcu)						\
			ct_cpuidle_exit();				\
		if (!is_retention)					\
			cpu_pm_exit();					\
	}								\
									\
	__ret ? -1 : idx;						\
})

#define CPU_PM_CPU_IDLE_ENTER(low_level_idle_enter, idx)	\
	__CPU_PM_CPU_IDLE_ENTER(low_level_idle_enter, idx, idx, 0, 0)

#define CPU_PM_CPU_IDLE_ENTER_RETENTION(low_level_idle_enter, idx)	\
	__CPU_PM_CPU_IDLE_ENTER(low_level_idle_enter, idx, idx, 1, 0)

#define CPU_PM_CPU_IDLE_ENTER_PARAM(low_level_idle_enter, idx, state)	\
	__CPU_PM_CPU_IDLE_ENTER(low_level_idle_enter, idx, state, 0, 0)

#define CPU_PM_CPU_IDLE_ENTER_PARAM_RCU(low_level_idle_enter, idx, state)	\
	__CPU_PM_CPU_IDLE_ENTER(low_level_idle_enter, idx, state, 0, 1)

#define CPU_PM_CPU_IDLE_ENTER_RETENTION_PARAM(low_level_idle_enter, idx, state)	\
	__CPU_PM_CPU_IDLE_ENTER(low_level_idle_enter, idx, state, 1, 0)

#define CPU_PM_CPU_IDLE_ENTER_RETENTION_PARAM_RCU(low_level_idle_enter, idx, state)	\
	__CPU_PM_CPU_IDLE_ENTER(low_level_idle_enter, idx, state, 1, 1)

#endif /* _LINUX_CPUIDLE_H */
