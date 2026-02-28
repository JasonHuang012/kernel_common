#  swap entry
当一个页面被换出时，Linux使用相应的 PTE 来存储足够的信息以再次在磁盘上定位该页面。显然，PTE 本身不足以精确存储页面在磁盘上的位置，但足以将索引存储到 swap_info中array 和swap_map中的偏移量。

巧妙的利用被换出页面的pte来确定swap area和swap offset

相关结构体
```C#
// include/linux/mm_types.h

typedef struct {
	unsigned long val;
} swp_entry_t;

```
```c#
// arch/arm64/include/asm/pgtable.h
/*
 * Encode and decode a swap entry:
 *      bits 0-1:       present (must be zero)  // 置0，表示为swap entry
 *      bits 2:         remember PG_anon_exclusive
 *      bit  3:         remember uffd-wp state
 *      bits 6-10:      swap type               // swap_info[]数组的索引，用于查找对应的swap area
 *      bit  11:        PTE_PRESENT_INVALID (must be zero)
 *      bits 12-61:     swap offset             // swap area或swap分区内的索引，用于查找对应slot
 */
#define __SWP_TYPE_SHIFT        6
#define __SWP_TYPE_BITS         5
#define __SWP_TYPE_MASK         ((1 << __SWP_TYPE_BITS) - 1)
#define __SWP_OFFSET_SHIFT      12
#define __SWP_OFFSET_BITS       50
#define __SWP_OFFSET_MASK       ((1UL << __SWP_OFFSET_BITS) - 1)

#define __swp_type(x)           (((x).val >> __SWP_TYPE_SHIFT) & __SWP_TYPE_MASK)
#define __swp_offset(x)         (((x).val >> __SWP_OFFSET_SHIFT) & __SWP_OFFSET_MASK)
#define __swp_entry(type,offset) ((swp_entry_t) { ((type) << __SWP_TYPE_SHIFT) | ((offset) << __SWP_OFFSET_SHIFT) })

#define __pte_to_swp_entry(pte) ((swp_entry_t) { pte_val(pte) })
#define __swp_entry_to_pte(swp) ((pte_t) { (swp).val })
```

页面首次swapout，比如A进程和B进程共同访问的匿名页
1.add_to_swap调用folio_alloc_swap分配空闲的slot，保存到entry中;
2.try_to_unmap->rmap_walk->rmap_walk_aon->try_to_unmap_one，利用反向映射，将进程A和B的PTE更新为swap entry

说明
- struct anon_vma保存了所有映射次匿名页面的vma链表，根据这个vma链表可以找找到对应进程的mm: vma->vm_mm
- 进程虚拟地址--> 页面解析--> pte --> 页面物理地址

swapout的entry保存
try_to_unmap -> try_to_unmap_one

swapin的entry取出及检索slot
do_swap_page -> pte_to_swp_entry

# swapcache

swapcache是交换缓存区，一方面与pagecache类似、用于提高性能，另一方面是避免频繁pageout/pagein，比如一个页面被swapout，加入了swapcache，还没开始写入swap分区时，这时有进程访问了这个页面，则可以直接从swapcache中命中。

swapcache使用场景
- swapout

- swapin
do_swap_page会先在swapcache查找，如果swapcache命中了，则直接使用；如果swapcache没有命中则会从swap分区加载，并加入到swapcache中；


- 加入swapcache
	- API
		- add_to_swap_cache	// 将页面加入swapcache
		- add_to_swap		// 分配alloc空间并加入swapcache(调用add_to_swap_cache)
	- 页面回收时，如果是可以swap的匿名页面，则尝试为其分配swap空间并加入swapcache
		- shrink_folio_list->add_to_swap
	- 匿名页面pageout时，会尝试先加入swapcache，如果可以加入成功，再尝试交换出去;
		- pageout->shmem_writepage->add_to_swap_cache->swap_writepage
	- 匿名页page fault时，如果需要从swap分区加载页面数据，如果走的是异步预加载流程，则会加入swapcache
		- __read_swap_cache_async
		- do_swap_page->swapin_readahead->swap_vma_readahead/swap_cluster_readahead->__read_swap_cache_async->add_to_swap_cache

- 移除swapcache
	- API
		- folio_free_swap
			- 有判断条件，需要条件成立才能移除swapcache;
				- 1.页面正在回写;
				- 2.没有进程PTE使用对应的swap entry;
				- 详细见函数解析，或者swap_writepage调用中说明;
		- delete_from_swap_cache;
			- 无判断条件，直接释放；
	- 内存回收，匿名页回收, shrink_folio_list->pageout->shmem_write_page->swap_writepage->folio_free_swap;
	- 内存回收，shrink_folio_list->激活的页面是swapcache而且swap缓存满了，则folio_free_swap;
	- swapoff, swapoff->try_to_unuse->folio_free_swap;
	- 回收swap, __try_to_reclaim_swap->folio_free_swap;

=====
内存压缩的调用路径中，add_to_swap并不会直接call pageout，add_to_swap只是将page变成SwapPage并和swap_entry建立对应关系；
并且，add_to_swap时，page对应的mapping还未unmap；
流程大致是：先add_to_swap，后try_to_unmap，再去call pageout；
以上基于kernel-5.4;

@bsp：swap & page fault流程
1:将待swap的page 加入到swapcache中；
2:解除task和该page的映射关系；
3:通过pageout 进行zram压缩，并将此page压到某个buffer中；
4:压缩完成后，选择性从swapcache中free该page（比如swapcache满了）；

第2步之后，task如果访问该page，则会触发pagefault；
如果该page还在swapcache中，则将task和该page重新建立映射关系即可，即minor fault；
如果该page不在swapcache中，则会重申请一个新的page，并通过zram解压缩、将之前压缩后的buffer解压到此page中，即major fault；
====

====
1. 在内存回收swap out路径上，在PTE被设置为swap entry之后，page真正被回收之前，可能被do_swap_page()这种page fault命中，命中时候我们需要查询到对应的page是哪一个，把它映射到进程的PTE里面；
2. 在swap in的路径上，往往可能多个进程指向同一个swap entry。比如a进程fork b进程，这样a和b仍然共享的内存被swap out出去之后，无论是a还是b，相应的PTE都指向了同样的swap entry。若a进程率先swap in，将swap in时候申请的page加入swapcache，则b进程同样位置发生do_swap_page()的时候，可以在swapcache查询到该page直接映射到PTE;
3. 在swap in的路径上do_swap_page，可能会做readahead，把page fault PTE周围的小部分区域提前从swap读入内存，这些预读的page加入swapcache。稍后等到预读区域的page fault真正发生的时候，do_swap_page()也可直接命中swapcache。
====

====
匿名页即将被swap-out时会先被放进swap cache，但通常只存在很短暂的时间，因为紧接着在pageout完成之后它就会从swap cache中删除，毕竟swap-out的目的就是为了腾出空闲内存；
【注：参见mm/vmscan.c: shrink_page_list()，它调用的add_to_swap()会把swap cache页面标记成dirty，然后它调用try_to_unmap()将页面对应的page table mapping都删除，再调用pageout()回写dirty page，最后try_to_free_swap()会把该页从swap cache中删除。】
曾经被swap-out现在又被swap-in的匿名页会在swap cache中，直到页面中的内容发生变化、或者原来用过的交换区空间被回收为止。
【注：当匿名页的内容发生变化时会删除对应的swap cache，代码参见mm/swapfile.c: reuse_swap_page()。】
SwapCached背后的含义是：系统中有多少匿名页曾经被swap-out、现在又被swap-in并且swap-in之后页面中的内容一直没发生变化。也就是说，如果这些匿名页需要被swap-out的话，是无需进行I/O write操作的。
====

## 应用场景

- swapin
如果有


page flag
PG_swapcache
## 相关API

# 重点API及结构体
## API
 - init_swap_address_space
```
int init_swap_address_space(unsigned int type, unsigned long nr_pages)
```
初始化swap空间对应的address_space，一个address_space对应一个64M的swap空间
新版内核用了swap table替换address space。

## 全局变量
- swapper_spaces
struct address_space *swapper_spaces[MAX_SWAPFILES] __read_mostly;

	swapper_spaces是一个全局数组，一个成员对应一个swap分区大小
	swapper_spaces数组的成员是一个address_space数组，长度为 swap分区大小/64M

static unsigned int nr_swapper_spaces[MAX_SWAPFILES] __read_mostly;


|-----------|----------------------|---------------------|-------|----------------------|
|swap分区1  | address_space1 [64M] |address_space2 [64M] | ......| address_spaceN [64M] |
|-----------|----------------------|---------------------|-------|----------------------|
|swap分区2  | address_space1 [64M] |address_space2 [64M] | ......| address_spaceN [64M] |
|-----------|----------------------|---------------------|-------|----------------------|


## 结构体
struct swap_info_struct

# 学习过程的问题问题
## 一个页面正在swapout的过程中，假设还没回写到swap分区，这时A进程访问了这个页面，这时候会如何处理？这个页面是被保存在swapcache中吗？将页面对应的地址更新到A进程的PTE?这个页面最后会被回写到swap分区吗？

-  这个页面是被保存在 swap cache 中吗？
是的，在换出过程中，页面会一直保留在 swap cache 中，直到换出完成或换出被取消。

- 将页面对应的地址更新到 A 进程的 PTE？
是的，缺页异常do_swap_page:
找到物理页面（通过 swap cache） -> 创建新的页表项指向物理页面 -> 更新 A 进程的页表

- 这个页面最后会被回写到 swap 分区吗？
取决于情况：
 * 如果页面在访问后被修改：会被标记为 dirty，需要重新换出
 * 如果页面保持干净：可能不需要立即换出，但会在内存压力时被再次考虑换出
 * 如果换出操作已经完成：数据已经在 swap 分区，但物理页面会重新被使用;
	- 也就是既被换出到swap中，又有一份在内存中；

- 关键点：
 * swap cache 作为协调层，防止重复换入换出
 * 锁机制确保数据一致性
 * 页面被重新激活，推迟换出操作
 * 按需换入，只有真正访问时才从 swap 读回数据
这种设计确保了内存管理的效率和数据的一致性。

## swap entry不是通过pte得到的吗？为什么folio_alloc_swap看起来是在找空闲的slot，而不是通过page的pte来确定slot
开始学习swap的时候，以为在swapout时，是根据物理页面的pte来确定换出的swap slot，
其实是查找空闲的slot, 在解除进程页面映射时，利用rmap将所有相关进程页表中页面原来的pte修改为这个slot对应的swap entry，后续在swapin时，发现缺页异常，通过识别pte，发现是swap entry的pte，则先从swapcache里面找，如果找到了，直接修改pte为swapcache对应的物理页面，如果找不到，则从swap分区读出到新分配的内存页面，再修改pte执行新的内存页面；

## PTE是指是保存在哪里的？比如有两个进程A和B，他们都访问了同一个页面，这个页面对应的PTE是保存在哪里？是A和B各保存一份，还是共用一份？
每个进程都有自己独立的页表，用于做进程虚拟地址到物理地址的转换，如下:
进程虚拟地址--> pgd --> p4d --> pud -->  pmd --> pte --> 页面物理地址

PTE 的位域组成示例（简化）:
| 物理页帧号 (PFN) | 保留位 | 访问位 | 脏位 | 权限位 | 存在位 |
 63                  12 11     7 6      5 4     3 2      1 0
 所以每个进程都有独立的页表，用于保存pte，在页面swapout时，根据rmap机制，找到所有共享该的进程，修改页面对应的pte为swap entry。

# 将folio加入swapcache，究竟是将folio加入到哪里？folio的物理空间有没有发生变化？是将folio拷贝到另一块页面空间，再将原来的folio释放？还是将folio保存到xarray中、标记GP_swapcache？
folio加入swapcache，并不会改变folio原先的物理位置，也不需要拷贝，只是将folio保存到swapcache对应的xarray中。

# PG_swapcache 和 PG_swapbacked
PG_swapcache表示页面当前位于swap cache中，页面刚换出、还没写入swap分区，或者页面刚刚换入。
设置PG_swapcache: add_to_swap_cache， pageout时加入swap缓存、pagein时刚从swap分区加载到内存；
清除PG_swapcache: __delete_from_swap_cache，页面从swap cache写入到swap分区后，需要清除PG_swapcache，后续会释放该页面。

# zram swapin时swapcache使用流程
swapin流程
1. 通过 PTE 得到 swap entry (type=1, offset=0x1000)
2. 在 swap cache (xarray) 中查找: index=0x1000 → folio
   - 如果找到: 直接使用物理页面
   - 如果没找到:
3. 通过 zram_table[0x1000] 找到 zsmalloc handle
4. 从 zsmalloc 解压缩数据到新分配的物理页面
5. 将新页面加入 swap cache

# zram驱动分析
## zsmalloc

zsmalloc是为内存压缩zram而实现的一种内存分配器，用于存放被压缩后的页面数据，以一个page为4KB为例子，一个页面被压缩后的数据肯定小于4KB，根据压缩算法及数据的简繁程度的不同，可能一个页面被压缩后的数据小到几十个字节、也可能还是接近于4KB。
那为什么不直接使用slab分配呢？
我们知道slab是用于分配较小、且物理连续的内存块，slab将多个大小相同的内存块对象放在同一个内存页面中，但是有时候这些对象不能正好把整个内存页面占满，导致产生内部碎片而造成浪费。比如一个3072字节大小的对象，只能占据3/4的页面空间，导致剩余的1/4空间被浪费。以前的内核版本尝试为这种情况分配多个连续的物理页面来减少内存的浪费，比如对于3072字节的对象，分配3个连续的物理页面，这样刚好3个对象刚好可以占满这3个页面，而不会造成碎片浪费。但是对于低内存设备，有时候很难分配到多个连续的物理页面，特别是在系统运行较长时间后。
但是zram现在完全是由cpu实现，并不是一定要用物理连续的内存，只需要能映射为连续的虚拟内存即可，所以zsmalloc就产生了，zsmalloc像vmalloc一样申请的是多个物理不连续的页面(alloc_page())，唯一的区别是zsmalloc不要求马上为这些页面建立虚拟映射，因为考虑到32位系统虚拟地址空间有限，当使用object时再做映射(zs_map_object, 以页面为单位进行映射)，使用完object后解除映射(zs_unmap_object)。

slab是以单个物理页面为一个内存块来分割object，而zsmalloc是将多个离散的物理页面作为一个组合页面，称为zspage，用于存放各类大小相同的object。
zsmalloc和slab的kmalloc类似，也是有固定大小的object size，只不过zsmalloc比kmalloc有更多的粒度，4K页面为例子，有255种小于PAGE_SIZE的size，从32字节(ZS_MIN_ALLOC_SIZE)开始、每间隔16字节(ZS_SIZE_CLASS_DELTA)增加一个一类object。
zsmalloc的object也被称为zpage，一个object/zpage可以跨越两个物理页面，比如一个object大小是2/3页面，则两个page可以容纳三个object。

zsmalloc的使用流程：
zs_create_pool-->zs_malloc-->zs_map_object-->object读写-->zs_unmap_object-->zs_free-->zs_destory_pool


CONFIG_ZSMALLOC_CHAIN_SIZE


