/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_GENERIC_QRWLOCK_TYPES_H
#define __ASM_GENERIC_QRWLOCK_TYPES_H

#include <linux/types.h>
#include <asm/byteorder.h>
#include <asm/spinlock_types.h>

/*
 * The queued read/write lock data structure
 */

/* rwlock使用了qrwlock */
typedef struct qrwlock {
	union {
		atomic_t cnts;
		struct {
#ifdef __LITTLE_ENDIAN
			u8 wlocked;	/* Locked for write? */
			u8 __lstate[3];
#else
			u8 __lstate[3];
			u8 wlocked;	/* Locked for write? */
#endif
		};
	};
	arch_spinlock_t		wait_lock;
} arch_rwlock_t;

/*
   cnts 字段的位域编码（关键）

   bit 31              9   8        7           0
   +-------------------+---+--------+-----------+
   |   reader count    | W | (unused)  wlocked  |
   +-------------------+---+--------+-----------+
     [31:9] 读者计数      ↑               ↑
     _QR_BIAS=0x200    bit8           bits[7:0]
                    _QW_WAITING=0x100  _QW_LOCKED=0xff

  #define _QW_WAITING  0x100   // bit8：有写者在等待队列中
  #define _QW_LOCKED   0x0ff   // bits[7:0]：写者已持锁
  #define _QW_WMASK    0x1ff   // bits[8:0]：写者相关位掩码
  #define _QR_SHIFT    9       // 读者计数起始位
  #define _QR_BIAS     (1U << _QR_SHIFT)  // = 0x200，每个读者加这个值

  各状态的 cnts 值：

  ┌─────────────────┬────────────────────────────────────┐
  │     cnts 值     │                含义                │
  ├─────────────────┼────────────────────────────────────┤
  │ 0x00000000      │ 空闲                               │
  ├─────────────────┼────────────────────────────────────┤
  │ N * 0x200       │ N 个读者持锁                       │
  ├─────────────────┼────────────────────────────────────┤
  │ 0x000000ff      │ 写者持锁（无等待读者）             │
  ├─────────────────┼────────────────────────────────────┤
  │ 0x00000100      │ 写者在等待，无读者（写者即将获锁） │
  ├─────────────────┼────────────────────────────────────┤
  │ N*0x200 + 0x100 │ N 个读者持锁，一个写者在等待       │
  └─────────────────┴────────────────────────────────────┘


*/

#define	__ARCH_RW_LOCK_UNLOCKED {		\
	{ .cnts = ATOMIC_INIT(0), },		\
	.wait_lock = __ARCH_SPIN_LOCK_UNLOCKED,	\
}

#endif /* __ASM_GENERIC_QRWLOCK_TYPES_H */
