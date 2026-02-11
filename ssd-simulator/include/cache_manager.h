/**
 * @file cache_manager.h
 * @brief SSD Simulator - Cache Manager Header
 */

#ifndef __CACHE_MANAGER_H__
#define __CACHE_MANAGER_H__

#include <stdint.h>
#include <stdbool.h>
#include "nand_simulator.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 缓存条目状态 */
typedef enum {
    CACHE_ENTRY_FREE = 0,
    CACHE_ENTRY_VALID,
    CACHE_ENTRY_DIRTY,
    CACHE_ENTRY_FLUSHING,
} cache_entry_state_t;

/* 缓存统计 */
typedef struct cache_stats {
    uint64_t read_hits;
    uint64_t read_misses;
    uint64_t write_hits;
    uint64_t write_misses;
    uint64_t flushes;
    uint64_t evictions;
    double read_hit_rate;
    double write_hit_rate;
} cache_stats_t;

/* Opaque类型 */
typedef struct cache_manager cache_manager_t;

/**
 * @brief 创建缓存管理器
 * @param capacity 缓存容量（页数）
 * @param page_size 页大小
 * @return 管理器句柄，NULL失败
 */
cache_manager_t* cache_manager_create(uint32_t capacity, uint32_t page_size);

/**
 * @brief 销毁缓存管理器
 * @param cache 管理器句柄
 */
void cache_manager_destroy(cache_manager_t* cache);

/**
 * @brief 读缓存查找
 * @param cache 管理器句柄
 * @param lba 逻辑块地址
 * @param buf 输出缓冲区
 * @return 0命中，负数未命中
 */
int cache_read(cache_manager_t* cache, uint64_t lba, void* buf);

/**
 * @brief 写缓存
 * @param cache 管理器句柄
 * @param lba 逻辑块地址
 * @param buf 数据缓冲区
 * @return 0成功
 */
int cache_write(cache_manager_t* cache, uint64_t lba, void* buf);

/**
 * @brief 使能缓存条目
 * @param cache 管理器句柄
 * @param lba 逻辑块地址
 * @return 0成功
 */
int cache_invalidate(cache_manager_t* cache, uint64_t lba);

/**
 * @brief 刷新脏数据到NAND
 * @param cache 管理器句柄
 * @param lba 逻辑块地址（-1表示全部）
 * @return 刷新的页数
 */
uint32_t cache_flush(cache_manager_t* cache, int64_t lba);

/**
 * @brief 预读数据
 * @param cache 管理器句柄
 * @param lba 起始逻辑块地址
 * @param count 预读数量
 * @return 预读成功的页数
 */
uint32_t cache_prefetch(cache_manager_t* cache, uint64_t lba, uint32_t count);

/**
 * @brief 获取统计信息
 * @param cache 管理器句柄
 * @param stats 统计输出
 */
void cache_get_stats(cache_manager_t* cache, cache_stats_t* stats);

/**
 * @brief 打印缓存信息
 * @param cache 管理器句柄
 */
void cache_dump_info(cache_manager_t* cache);

#ifdef __cplusplus
}
#endif

#endif /* __CACHE_MANAGER_H__ */
