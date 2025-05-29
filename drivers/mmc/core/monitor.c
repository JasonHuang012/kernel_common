// SPDX-License-Identifier: GPL-2.0-only
/*
 *  linux/drivers/mmc/core/monitor.c
 */
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/timekeeping.h>
#include "monitor.h"

bool mmc_monitor_inited = 0;
bool mmc_monitor_start = 0;
struct mmc_monitor_data *mmc_monitor_p = NULL;
struct proc_dir_entry *mmc_monitor_proc = NULL;

static int mmc_monitor_proc_show(struct seq_file *m, void *v)
{
	time64_t mmc_monitor_time = 0;
	u64 mmc_read_per_min, mmc_write_per_min, min;


	if (mmc_monitor_p->mmc_monitor_time_stop)
		mmc_monitor_time = mmc_monitor_p->mmc_monitor_time_stop - mmc_monitor_p->mmc_monitor_time_start;
	else
		mmc_monitor_time = ktime_get_real_seconds() - mmc_monitor_p->mmc_monitor_time_start;

	seq_printf(m, "mmc monitor, time: %lld sec\n", mmc_monitor_time);

	min = mmc_monitor_time;
	do_div(min, 60);
	mmc_read_per_min = mmc_monitor_p->mmc_read_cnt;
	do_div(mmc_read_per_min, min);
	mmc_write_per_min = mmc_monitor_p->mmc_write_cnt;
	do_div(mmc_write_per_min, min);
	seq_printf(m, "read  total count: %u, per min: %llu\n", mmc_monitor_p->mmc_read_cnt, mmc_read_per_min);
	seq_printf(m, "write total count: %u, per min: %llu\n", mmc_monitor_p->mmc_write_cnt, mmc_write_per_min);

	return 0;
}

static int mmc_monitor_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, mmc_monitor_proc_show, NULL);
}

static ssize_t mmc_monitor_proc_write(struct file *file, const char __user *buffer,
		size_t count, loff_t *ppos)
{
	char cmd_str[4] = {0};
	int cmd = 0;

	if (copy_from_user(cmd_str, buffer, count))
		return -EFAULT;

	cmd = simple_strtoul(cmd_str, NULL, 10);
	if (cmd) {
		if (mmc_monitor_start)
			return count;

		if (mmc_monitor_p == NULL) {
			mmc_monitor_p = kzalloc(sizeof(*mmc_monitor_p), GFP_KERNEL);
			spin_lock_init(&mmc_monitor_p->read_lock);
			spin_lock_init(&mmc_monitor_p->write_lock);
		}
		mmc_monitor_p->mmc_read_cnt = 0;
		mmc_monitor_p->mmc_write_cnt = 0;
		mmc_monitor_p->mmc_monitor_time_start = ktime_get_real_seconds();
		mmc_monitor_p->mmc_monitor_time_stop = 0;
		mmc_monitor_start = 1;
	} else {
		if (!mmc_monitor_start)
			return count;

		mmc_monitor_start = 0;
		mmc_monitor_p->mmc_monitor_time_stop = ktime_get_real_seconds();
	}

	return count;
}

static const struct proc_ops mmc_monitor_proc_ops = {
	.proc_open	= mmc_monitor_proc_open,
	.proc_read	= seq_read,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
	.proc_write	= mmc_monitor_proc_write,
};

void mmc_read_write_monitor(u32 opcode)
{
	if (mmc_monitor_start) {
		if (opcode == 24 || opcode == 25) {
			spin_lock(&mmc_monitor_p->write_lock);
			mmc_monitor_p->mmc_write_cnt++;
			spin_unlock(&mmc_monitor_p->write_lock);
		} else if (opcode == 17 || opcode == 18) {
			spin_lock(&mmc_monitor_p->read_lock);
			mmc_monitor_p->mmc_read_cnt++;
			spin_unlock(&mmc_monitor_p->read_lock);
		}
	}

	return;
}

void mmc_monitor_init(void)
{
	if (!mmc_monitor_inited) {
		mmc_monitor_inited = 1;
		mmc_monitor_proc = proc_create_data("mmc_monitor", 0660, NULL,
					&mmc_monitor_proc_ops, "mmc_monitor");
	}
}

