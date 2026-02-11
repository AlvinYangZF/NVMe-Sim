/**
 * @file ddr_manager.h
 * @brief SSD Simulator - DDR Memory Manager Header
 */

#ifndef __DDR_MANAGER_H__
#define __DDR_MANAGER_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 内存块大小等级 */
typedef enum {
    MEM_BLOCK_4K = 0,       /* 4KB   - 小IO, 元数据 */
    MEM_BLOCK_16K,          /* 16KB  - NAND页大小 */
    MEM_BLOCK_64K,          /* 64KB  - 大IO聚合 */
    MEM_BLOCK_256K,         /* 256KB - 映射表块 */
    MEM_BLOCK_1M,           /* 1MB   - 大缓冲区 */
    MEM_BLOCK_4M,           /* 4MB   - 超大IO */
    MEM_BLOCK_NUM
} mem_block_size_t;

/* 内存块标志 */
#define MEM_FLAG_ZERO       (1 << 0)    /* 清零 */
#define MEM_FLAG_PINNED     (1 << 1)    /* 固定 (不释放) */
#define MEM_FLAG_CONTIG     (1 << 2)    /* 物理连续 */

/* 数据缓冲区 */
typedef struct data_buffer {
    void* virtual_addr;         /* 虚拟地址 */
    uint64_t physical_addr;     /* 模拟物理地址 */
    size_t size;                /* 大小 */
    uint32_t ref_count;         /* 引用计数 */
    
    /* 链表 */
    struct data_buffer* next;
    struct data_buffer* prev;
} data_buffer_t;

/* 缓冲区池 */
typedef struct buffer_pool {
    const char* name;
    size_t buffer_size;
    uint32_t num_buffers;
    
    data_buffer_t* buffers;
    data_buffer_t* free_list;
    uint32_t free_count;
    
    pthread_spinlock_t lock;
    
    /* 统计 */
    uint64_t hits;
    uint64_t misses;
} buffer_pool_t;

/* DDR统计 */
typedef struct ddr_stats {
    uint64_t total_allocs;
    uint64_t total_frees;
    uint64_t current_used;
    uint64_t peak_used;
    uint64_t alloc_failures;
    
    /* 按大小等级统计 */
    struct {
        uint64_t allocs;
        uint64_t frees;
        uint64_t current;
    } slab_stats[MEM_BLOCK_NUM];
} ddr_stats_t;

/**
 * @brief 初始化DDR管理器
 * @param capacity 总容量 (字节)
 * @return 0成功，负数错误码
 */
int ddr_manager_init(uint64_t capacity);

/**
 * @brief 销毁DDR管理器
 */
void ddr_manager_destroy(void);

/**
 * @brief 分配内存
 * @param size 大小
 * @param align 对齐要求 (必须是2的幂)
 * @return 内存指针，NULL失败
 */
void* ddr_alloc(size_t size, size_t align);

/**
 * @brief 分配内存（带标志）
 * @param size 大小
 * @param align 对齐要求
 * @param flags 标志
 * @return 内存指针，NULL失败
 */
void* ddr_alloc_flags(size_t size, size_t align, uint32_t flags);

/**
 * @brief 释放内存
 * @param ptr 内存指针
 */
void ddr_free(void* ptr);

/**
 * @brief 重新分配
 * @param ptr 原指针
 * @param new_size 新大小
 * @return 新指针，NULL失败
 */
void* ddr_realloc(void* ptr, size_t new_size);

/**
 * @brief 增加引用计数
 * @param ptr 内存指针
 */
void ddr_ref(void* ptr);

/**
 * @brief 减少引用计数
 * @param ptr 内存指针
 */
void ddr_unref(void* ptr);

/**
 * @brief 从缓冲区池获取缓冲区
 * @param pool_name 池名称 ("data", "meta", "map")
 * @return 缓冲区指针，NULL失败
 */
data_buffer_t* ddr_buffer_acquire(const char* pool_name);

/**
 * @brief 释放缓冲区回池
 * @param buf 缓冲区指针
 */
void ddr_buffer_release(data_buffer_t* buf);

/**
 * @brief 虚拟地址转物理地址
 * @param virt 虚拟地址
 * @return 物理地址
 */
uint64_t ddr_virt_to_phys(void* virt);

/**
 * @brief 物理地址转虚拟地址
 * @param phys 物理地址
 * @return 虚拟地址
 */
void* ddr_phys_to_virt(uint64_t phys);

/**
 * @brief 获取统计信息
 * @param stats 统计输出
 */
void ddr_get_stats(ddr_stats_t* stats);

/**
 * @brief 打印内存使用信息
 */
void ddr_dump_usage(void);

/* 便利宏 */
#define ddr_alloc_page() ddr_alloc(4096, 4096)
#define ddr_alloc_nand_page() ddr_alloc(16384, 16384)
#define ddr_zalloc(size) ddr_alloc_flags(size, 64, MEM_FLAG_ZERO)

#ifdef __cplusplus
}
#endif

#endif /* __DDR_MANAGER_H__ */
