/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 ARM Ltd.
 */
#ifndef __ASM_VDSO_PROCESSOR_H
#define __ASM_VDSO_PROCESSOR_H

#ifndef __ASSEMBLY__

/*
 * yield指令用来告知硬件系统，本cpu上执行的指令是polling操作，没有那么急迫，
 * 如果有任何的资源冲突，本cpu可以让出控制权。
 * CPU 松弛下来，降低功耗，把资源配置给其他thread等,CPU 松弛下来，降低功耗，把资源配置给其他thread.
 */
static inline void cpu_relax(void)
{
	asm volatile("yield" ::: "memory");
}

#endif /* __ASSEMBLY__ */

#endif /* __ASM_VDSO_PROCESSOR_H */
