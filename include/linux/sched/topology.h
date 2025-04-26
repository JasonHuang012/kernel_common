/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_SCHED_TOPOLOGY_H
#define _LINUX_SCHED_TOPOLOGY_H

#include <linux/topology.h>

#include <linux/sched/idle.h>

/*
 * sched-domains (multiprocessor balancing) declarations:
 */
#ifdef CONFIG_SMP

/* Generate SD flag indexes */
#define SD_FLAG(name, mflags) __##name,
enum {
	#include <linux/sched/sd_flags.h>
	__SD_FLAG_CNT,
};
#undef SD_FLAG
/* Generate SD flag bits */
#define SD_FLAG(name, mflags) name = 1 << __##name,
enum {
	#include <linux/sched/sd_flags.h>
};
#undef SD_FLAG

#ifdef CONFIG_SCHED_DEBUG

struct sd_flag_debug {
	unsigned int meta_flags;
	char *name;
};
extern const struct sd_flag_debug sd_flag_debug[];

#endif

#ifdef CONFIG_SCHED_SMT
static inline int cpu_smt_flags(void)
{
	return SD_SHARE_CPUCAPACITY | SD_SHARE_LLC;
}
#endif

#ifdef CONFIG_SCHED_CLUSTER
static inline int cpu_cluster_flags(void)
{
	return SD_CLUSTER | SD_SHARE_LLC;
}
#endif

#ifdef CONFIG_SCHED_MC
static inline int cpu_core_flags(void)
{
	return SD_SHARE_LLC;
}
#endif

#ifdef CONFIG_NUMA
static inline int cpu_numa_flags(void)
{
	return SD_NUMA;
}
#endif

extern int arch_asym_cpu_priority(int cpu);

struct sched_domain_attr {
	int relax_domain_level;
};

#define SD_ATTR_INIT	(struct sched_domain_attr) {	\
	.relax_domain_level = -1,			\
}

extern int sched_domain_level_max;

struct sched_group;

struct sched_domain_shared {
	atomic_t	ref;
	atomic_t	nr_busy_cpus;
	int		has_idle_cores;
	int		nr_idle_scan;
};

/*
 * 调度域表示cpu物理拓扑结构中的层级关系，调度组是负载均衡的基本单位
 * 一个调度域包含多个调度组，系统做负载均衡时，先保证一个调度域中的所有调度组的负载均衡，再考虑跨域的负载均衡 
 */
struct sched_domain {
	/* These fields must be setup */
	/* base domain的child为NULL，root domain的parent为NULL */
	struct sched_domain __rcu *parent;	/* top domain must be null terminated */
	struct sched_domain __rcu *child;	/* bottom domain must be null terminated */
	/* 本调度域中的调度组，形成一个环形链表, groups为链表头 */
	struct sched_group *groups;	/* the balancing groups of the domain */
	/* 检查负载均衡的最小时间间隔，检查过于频繁会带来额外的系统开销 */
	unsigned long min_interval;	/* Minimum balance interval ms */
	/* 检查负载均衡的最小时间间隔，太长时间不检查会导致负载差异过大 */
	unsigned long max_interval;	/* Maximum balance interval ms */
	/*
	 * 反映cpu忙碌程度的参数，系统会根据实际运行情况动态调整cpu负载均衡的时间间隔，该值记录在balance_interval字段中
	 * 如果cpu很繁忙，时间间隔就适当延长一点: busy_factor * balance_interval
	 */
	unsigned int busy_factor;	/* less balancing by factor if busy */
	/* 表示负载不均衡的阈值，调度域内的不均衡状态达到一定程度后就开始执行负载均衡 */
	unsigned int imbalance_pct;	/* No balance until over watermark */
	/* 和nr_balance_failed配合控制负载均衡中的迁移力度，当nr_balance_failed大于cache_nice_tries时，负载均衡会更加激进 */
	unsigned int cache_nice_tries;	/* Leave cache hot tasks for # tries */
	unsigned int imb_numa_nr;	/* Nr running tasks that allows a NUMA imbalance */

	/*
	 * nohz idle状态
	 * 1表示进入nohz idle
	 * 0表示退出idle，比如从nohz idle状态退出进行nohz idle balance_interval
	 */
	int nohz_idle;			/* NOHZ IDLE status */
	int flags;			/* See SD_* */
	/* 当前调度域在整个调度层级结构中的level,比如base调度域的level为0，向上依次加1,可以理解为调度域在树中的高度 */
	int level;

	/* Runtime fields. */
	/*
	 * 上一次做负载均衡的时间点，单位是jiffies
	 * 通过基础均衡时间间隔和当前sd的状态可以计算最终的均衡间隔时间（get_sd_balance_interval），
	 * last_balance加上这个计算得到的均衡时间间隔就是下一次均衡的时间点。
	 */
	unsigned long last_balance;	/* init to jiffies. units in jiffies */
	/* 负载均衡的时间间隔，会随着系统的运行而变化 */
	unsigned int balance_interval;	/* initialise to 1. units in ms. */
	/* 负载均衡失败的次数统计，当失败次数大于cache_nice_tries的时候，我们考虑迁移cache hot的任务，进行更激进的均衡操作 */
	unsigned int nr_balance_failed; /* initialise to 0 */

	/* idle_balance() stats */
	/*
	 * 在该domain上进行newidle balance的最大时间长度（即newidle balance的开销）。
	 * 最小值是sysctl_sched_migration_cost，是一个时间长度
	 */
	u64 max_newidle_lb_cost;
	/*
	 * 记录最近在该sched domain上进行newidle balance的最近时刻，是一个时间点
	 * 上面的max_newidle_lb_cost不是一成不变的，它有一个衰减过程，每秒衰减1%，这个成员就是用来控制衰减的
	 */
	unsigned long last_decay_max_lb_cost;

#ifdef CONFIG_SCHEDSTATS
	/* sched_balance_rq() stats */
	unsigned int lb_count[CPU_MAX_IDLE_TYPES];
	unsigned int lb_failed[CPU_MAX_IDLE_TYPES];
	unsigned int lb_balanced[CPU_MAX_IDLE_TYPES];
	unsigned int lb_imbalance[CPU_MAX_IDLE_TYPES];
	unsigned int lb_gained[CPU_MAX_IDLE_TYPES];
	unsigned int lb_hot_gained[CPU_MAX_IDLE_TYPES];
	unsigned int lb_nobusyg[CPU_MAX_IDLE_TYPES];
	unsigned int lb_nobusyq[CPU_MAX_IDLE_TYPES];

	/* Active load balancing */
	unsigned int alb_count;
	unsigned int alb_failed;
	unsigned int alb_pushed;

	/* SD_BALANCE_EXEC stats */
	unsigned int sbe_count;
	unsigned int sbe_balanced;
	unsigned int sbe_pushed;

	/* SD_BALANCE_FORK stats */
	unsigned int sbf_count;
	unsigned int sbf_balanced;
	unsigned int sbf_pushed;

	/* try_to_wake_up() stats */
	unsigned int ttwu_wake_remote;
	unsigned int ttwu_move_affine;
	unsigned int ttwu_move_balance;
#endif
#ifdef CONFIG_SCHED_DEBUG
	char *name;
#endif
	union {
		void *private;		/* used during construction */
		struct rcu_head rcu;	/* used during destruction */
	};
	/*
	 * 为了降低锁竞争， sched domain是per-cpu的
	 * 然而有些信息是需要在per-cpu的sched domain之间共享的，不能在每个sched_domain上构建
	 * 这些信息包括： 该sched domian中的busy cusy个数、是否有idle的cpu
	 */
	struct sched_domain_shared *shared;

	/* 当前调度域有多少个cpu */
	unsigned int span_weight;
	/*
	 * Span of all CPUs in this domain.
	 *
	 * NOTE: this field is variable length. (Allocated dynamically
	 * by attaching extra space to the end of the structure,
	 * depending on how many CPUs the kernel has booted up with)
	 */
	/* 当前调度域包含了哪些cpu，父调度域的span应该是所有子调度域的超集 */
	unsigned long span[];
};

static inline struct cpumask *sched_domain_span(struct sched_domain *sd)
{
	return to_cpumask(sd->span);
}

extern void partition_sched_domains_locked(int ndoms_new,
					   cpumask_var_t doms_new[],
					   struct sched_domain_attr *dattr_new);

extern void partition_sched_domains(int ndoms_new, cpumask_var_t doms_new[],
				    struct sched_domain_attr *dattr_new);

/* Allocate an array of sched domains, for partition_sched_domains(). */
cpumask_var_t *alloc_sched_domains(unsigned int ndoms);
void free_sched_domains(cpumask_var_t doms[], unsigned int ndoms);

bool cpus_equal_capacity(int this_cpu, int that_cpu);
bool cpus_share_cache(int this_cpu, int that_cpu);
bool cpus_share_resources(int this_cpu, int that_cpu);

typedef const struct cpumask *(*sched_domain_mask_f)(int cpu);
typedef int (*sched_domain_flags_f)(void);

#define SDTL_OVERLAP	0x01

struct sd_data {
	struct sched_domain *__percpu *sd;
	struct sched_domain_shared *__percpu *sds;
	struct sched_group *__percpu *sg;
	struct sched_group_capacity *__percpu *sgc;
};

struct sched_domain_topology_level {
	sched_domain_mask_f mask;
	sched_domain_flags_f sd_flags;
	int		    flags;
	int		    numa_level;
	struct sd_data      data;
#ifdef CONFIG_SCHED_DEBUG
	char                *name;
#endif
};

extern void __init set_sched_topology(struct sched_domain_topology_level *tl);

#ifdef CONFIG_SCHED_DEBUG
# define SD_INIT_NAME(type)		.name = #type
#else
# define SD_INIT_NAME(type)
#endif

#else /* CONFIG_SMP */

struct sched_domain_attr;

static inline void
partition_sched_domains_locked(int ndoms_new, cpumask_var_t doms_new[],
			       struct sched_domain_attr *dattr_new)
{
}

static inline void
partition_sched_domains(int ndoms_new, cpumask_var_t doms_new[],
			struct sched_domain_attr *dattr_new)
{
}

static inline bool cpus_equal_capacity(int this_cpu, int that_cpu)
{
	return true;
}

static inline bool cpus_share_cache(int this_cpu, int that_cpu)
{
	return true;
}

static inline bool cpus_share_resources(int this_cpu, int that_cpu)
{
	return true;
}

#endif	/* !CONFIG_SMP */

#if defined(CONFIG_ENERGY_MODEL) && defined(CONFIG_CPU_FREQ_GOV_SCHEDUTIL)
extern void rebuild_sched_domains_energy(void);
#else
static inline void rebuild_sched_domains_energy(void)
{
}
#endif

#ifndef arch_scale_cpu_capacity
/**
 * arch_scale_cpu_capacity - get the capacity scale factor of a given CPU.
 * @cpu: the CPU in question.
 *
 * Return: the CPU scale factor normalized against SCHED_CAPACITY_SCALE, i.e.
 *
 *             max_perf(cpu)
 *      ----------------------------- * SCHED_CAPACITY_SCALE
 *      max(max_perf(c) : c \in CPUs)
 */
static __always_inline
unsigned long arch_scale_cpu_capacity(int cpu)
{
	return SCHED_CAPACITY_SCALE;
}
#endif

#ifndef arch_scale_hw_pressure
static __always_inline
unsigned long arch_scale_hw_pressure(int cpu)
{
	return 0;
}
#endif

#ifndef arch_update_hw_pressure
static __always_inline
void arch_update_hw_pressure(const struct cpumask *cpus,
				  unsigned long capped_frequency)
{ }
#endif

#ifndef arch_scale_freq_ref
static __always_inline
unsigned int arch_scale_freq_ref(int cpu)
{
	return 0;
}
#endif

static inline int task_node(const struct task_struct *p)
{
	return cpu_to_node(task_cpu(p));
}

#endif /* _LINUX_SCHED_TOPOLOGY_H */
