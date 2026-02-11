/**
 * @file l2p_manager.h
 * @brief SSD Simulator - L2P (Logical to Physical) Address Mapping Header
 */

#ifndef __L2P_MANAGER_H__
#define __L2P_MANAGER_H__

#include <stdint.h>
#include <stdbool.h>
#include "nand_simulator.h"

#ifdef __cplusplus
extern "C" {
#endif

/* L2P标志 */
#define L2P_FLAG_VALID      (1 << 0)
#define L2P_FLAG_INVALID    (1 << 1)
#define L2P_FLAG_WRITING    (1 << 2)
#define L2P_FLAG_GC         (1 << 3)

/* L2P映射条目 */
typedef struct l2p_entry {
    uint64_t ppa;           /* 物理页地址 */
    uint16_t flags;         /* 状态标志 */
} l2p_entry_t;

/* 无效PPA */
#define L2P_INVALID_PPA     INVALID_PPA

/* L2P缓存统计 */
typedef struct l2p_stats {
    uint64_t cache_hits;
    uint64_t cache_misses;
    uint64_t total_lookups;
    uint64_t total_updates;
    double hit_rate;
    uint32_t cache_entries;
} l2p_stats_t;

/* Opaque类型 */
typedef struct l2p_manager l2p_manager_t;

/**
 * @brief 初始化L2P管理器
 * @param num_lbas 总LBA数量
 * @param cache_size 缓存大小（条目数）
 * @return 管理器句柄，NULL失败
 */
l2p_manager_t* l2p_manager_create(uint64_t num_lbas, uint32_t cache_size);

/**
 * @brief 销毁L2P管理器
 * @param l2p 管理器句柄
 */
void l2p_manager_destroy(l2p_manager_t* l2p);

/**
 * @brief L2P查找
 * @param l2p 管理器句柄
 * @param lba 逻辑块地址
 * @param entry 映射条目输出
 * @return 0成功，负数错误码
 */
int l2p_lookup(l2p_manager_t* l2p, uint64_t lba, l2p_entry_t* entry);

/**
 * @brief 批量L2P查找
 * @param l2p 管理器句柄
 * @param lbas LBA数组
 * @param entries 映射条目数组输出
 * @param count 数量
 * @return 成功查找的数量
 */
uint32_t l2p_lookup_batch(l2p_manager_t* l2p, uint64_t* lbas, 
                          l2p_entry_t* entries, uint32_t count);

/**
 * @brief L2P更新
 * @param l2p 管理器句柄
 * @param lba 逻辑块地址
 * @param ppa 物理页地址
 * @param flags 标志
 * @return 0成功，负数错误码
 */
int l2p_update(l2p_manager_t* l2p, uint64_t lba, uint64_t ppa, uint16_t flags);

/**
 * @brief 批量L2P更新
 * @param l2p 管理器句柄
 * @param lbas LBA数组
 * @param ppas PPA数组
 * @param flags 标志数组（可为NULL）
 * @param count 数量
 * @return 成功更新的数量
 */
uint32_t l2p_update_batch(l2p_manager_t* l2p, uint64_t* lbas, uint64_t* ppas,
                          uint16_t* flags, uint32_t count);

/**
 * @brief 使能LBA映射
 * @param l2p 管理器句柄
 * @param lba 逻辑块地址
 * @return 0成功
 */
int l2p_invalidate(l2p_manager_t* l2p, uint64_t lba);

/**
 * @brief 刷新缓存到NAND
 * @param l2p 管理器句柄
 * @return 0成功
 */
int l2p_flush(l2p_manager_t* l2p);

/**
 * @brief 分配空闲PPA
 * @param l2p 管理器句柄
 * @param ppa PPA输出
 * @return 0成功
 */
int l2p_alloc_ppa(l2p_manager_t* l2p, uint64_t* ppa);

/**
 * @brief 释放PPA
 * @param l2p 管理器句柄
 * @param ppa 物理页地址
 * @return 0成功
 */
int l2p_free_ppa(l2p_manager_t* l2p, uint64_t ppa);

/**
 * @brief 获取统计信息
 * @param l2p 管理器句柄
 * @param stats 统计输出
 */
void l2p_get_stats(l2p_manager_t* l2p, l2p_stats_t* stats);

/**
 * @brief 打印L2P信息
 * @param l2p 管理器句柄
 */
void l2p_dump_info(l2p_manager_t* l2p);

#ifdef __cplusplus
}
#endif

#endif /* __L2P_MANAGER_H__ */
