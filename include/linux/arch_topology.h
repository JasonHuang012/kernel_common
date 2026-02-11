/* SPDX-License-Identifier: GPL-2.0 */
/*
 * include/linux/arch_topology.h - arch specific cpu topology information
 */
#ifndef _LINUX_ARCH_TOPOLOGY_H_
#define _LINUX_ARCH_TOPOLOGY_H_

#include <linux/types.h>
#include <linux/percpu.h>

void topology_normalize_cpu_scale(void);
int topology_update_cpu_topology(void);

struct device_node;
bool topology_parse_cpu_capacity(struct device_node *cpu_node, int cpu);

DECLARE_PER_CPU(unsigned long, cpu_scale);

static inline unsigned long topology_get_cpu_scale(int cpu)
{
	return per_cpu(cpu_scale, cpu);
}

void topology_set_cpu_scale(unsigned int cpu, unsigned long capacity);

DECLARE_PER_CPU(unsigned long, capacity_freq_ref);

static inline unsigned long topology_get_freq_ref(int cpu)
{
	return per_cpu(capacity_freq_ref, cpu);
}

DECLARE_PER_CPU(unsigned long, arch_freq_scale);

static inline unsigned long topology_get_freq_scale(int cpu)
{
	return per_cpu(arch_freq_scale, cpu);
}

void topology_set_freq_scale(const struct cpumask *cpus, unsigned long cur_freq,
			     unsigned long max_freq);
bool topology_scale_freq_invariant(void);

enum scale_freq_source {
	SCALE_FREQ_SOURCE_CPUFREQ = 0,
	SCALE_FREQ_SOURCE_ARCH,
	SCALE_FREQ_SOURCE_CPPC,
};

struct scale_freq_data {
	enum scale_freq_source source;
	void (*set_freq_scale)(void);
};

void topology_scale_freq_tick(void);
void topology_set_scale_freq_source(struct scale_freq_data *data, const struct cpumask *cpus);
void topology_clear_scale_freq_source(enum scale_freq_source source, const struct cpumask *cpus);

DECLARE_PER_CPU(unsigned long, hw_pressure);

static inline unsigned long topology_get_hw_pressure(int cpu)
{
	return per_cpu(hw_pressure, cpu);
}

void topology_update_hw_pressure(const struct cpumask *cpus,
				      unsigned long capped_freq);

/*
 * cpu拓扑结构体，用于描述多核cpu的层次化架构
 *	- 向调度器提供cpu信息，用于调度任务；
 *	- 向用户提供cpu结构信息;
 */
struct cpu_topology {
	int thread_id;			// 线程级ID，SMT的逻辑cpu core
	int core_id;			// 物理cpu core ID
	int cluster_id;			// cluster ID，NUMA或缓存簇
	int package_id;			// 封装级ID，物理cpu插槽
	cpumask_t thread_sibling;	// 同一个物理core的逻辑cpu
	cpumask_t core_sibling;		// 同一cluster的物理cpu core
	cpumask_t cluster_sibling;	// 同一封装的cluster
	cpumask_t llc_sibling;		// 共享最后一级缓存的cpu
};

sched_smt_active
/*
id示例：
Package 0 (物理CPU插槽)
├── Cluster 0 (LLC共享域)
│   ├── Core 0
│   │   ├── Thread 0 (CPU0)  # thread_id=0, core_id=0, cluster_id=0, package_id=0
│   │   └── Thread 1 (CPU8)  # thread_id=1, core_id=0, cluster_id=0, package_id=0
│   ├── Core 1
│   │   ├── Thread 0 (CPU1)  # thread_id=0, core_id=1, cluster_id=0, package_id=0
│   │   └── Thread 1 (CPU9)  # thread_id=1, core_id=1, cluster_id=0, package_id=0
└── Cluster 1 (另一个LLC域)
    ├── Core 4
    │   ├── Thread 0 (CPU4)  # thread_id=0, core_id=4, cluster_id=1, package_id=0
    │   └──


cpumask示例:
// 对于CPU0（8核16线程系统）：
thread_sibling = 0x0101    // CPU0和CPU8（同一核心的两个超线程）
core_sibling   = 0x0F0F    // Core 0-3的所有线程（同一簇）
cluster_sibling= 0xFFFF    // 所有CPU（单簇系统）
llc_sibling    = 0x0F0F    // 共享LLC的CPU（Core 0-3）
*/

#ifdef CONFIG_GENERIC_ARCH_TOPOLOGY
extern struct cpu_topology cpu_topology[NR_CPUS];

#define topology_physical_package_id(cpu)	(cpu_topology[cpu].package_id)
#define topology_cluster_id(cpu)	(cpu_topology[cpu].cluster_id)
#define topology_core_id(cpu)		(cpu_topology[cpu].core_id)
#define topology_core_cpumask(cpu)	(&cpu_topology[cpu].core_sibling)
#define topology_sibling_cpumask(cpu)	(&cpu_topology[cpu].thread_sibling)
#define topology_cluster_cpumask(cpu)	(&cpu_topology[cpu].cluster_sibling)
#define topology_llc_cpumask(cpu)	(&cpu_topology[cpu].llc_sibling)
void init_cpu_topology(void);
void store_cpu_topology(unsigned int cpuid);
const struct cpumask *cpu_coregroup_mask(int cpu);
const struct cpumask *cpu_clustergroup_mask(int cpu);
void update_siblings_masks(unsigned int cpu);
void remove_cpu_topology(unsigned int cpuid);
void reset_cpu_topology(void);
int parse_acpi_topology(void);
void freq_inv_set_max_ratio(int cpu, u64 max_rate);
#endif
extern bool topology_update_done;

#endif /* _LINUX_ARCH_TOPOLOGY_H_ */
