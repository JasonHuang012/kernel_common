/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *  linux/drivers/mmc/core/monitor.h
 */
#ifndef _MMC_CORE_MONITOR_H
#define _MMC_CORE_MONITOR_H

struct mmc_monitor_data {
	spinlock_t read_lock;
	spinlock_t write_lock;
	u32 mmc_read_cnt;
	u32 mmc_write_cnt;
	time64_t mmc_monitor_time_start;
	time64_t mmc_monitor_time_stop;
};

void mmc_monitor_init(void);
void mmc_read_write_monitor(u32 opcode);

#endif
