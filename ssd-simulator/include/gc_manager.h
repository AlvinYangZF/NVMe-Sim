/**
 * @file gc_manager.h
 * @brief SSD Simulator - Garbage Collection Manager Header
 */

#ifndef __GC_MANAGER_H__
#define __GC_MANAGER_H__

#include <stdint.h>
#include <stdbool.h>
#include "nand_simulator.h"
#include "l2p_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* GC策略 */
typedef enum {
    GC_POLICY_GREEDY = 0,       /* 贪心：选择无效页最多的块 */
    GC_POLICY_COST_BENEFIT,     /* 成本收益 */
    GC_POLICY_WEAR_AWARE,       /* 考虑磨损均衡 */
} gc_policy_t;

/* GC统计 */
typedef struct gc_stats {
    uint64_t gc_count;          /* GC执行次数 */
    uint64_t blocks_reclaimed;  /* 回收的块数 */
    uint64_t pages_migrated;    /* 迁移的页数 */
    uint64_t bytes_migrated;    /* 迁移的字节数 */
    uint64_t total_gc_time_ms;  /* 总GC时间 */
    double avg_gc_time_ms;      /* 平均GC时间 */
    uint32_t current_victim_blocks; /* 当前候选块数 */
} gc_stats_t;

/* GC配置 */
typedef struct gc_config {
    gc_policy_t policy;         /* GC策略 */
    uint32_t threshold;         /* GC触发阈值（无效页百分比） */
    uint32_t min_free_blocks;   /* 最小空闲块数 */
    uint32_t max_victim_blocks; /* 最大候选块数 */
    uint8_t  enable_background; /* 启用后台GC */
} gc_config_t;

/* Opaque类型 */
typedef struct gc_manager gc_manager_t;

/**
 * @brief 创建GC管理器
 * @param nand NAND模拟器
 * @param l2p L2P管理器
 * @param config GC配置
 * @return 管理器句柄，NULL失败
 */
gc_manager_t* gc_manager_create(nand_simulator_t* nand, 
                                 l2p_manager_t* l2p,
                                 gc_config_t* config);

/**
 * @brief 销毁GC管理器
 * @param gc 管理器句柄
 */
void gc_manager_destroy(gc_manager_t* gc);

/**
 * @brief 触发GC
 * @param gc 管理器句柄
 * @param force 强制GC（忽略阈值）
 * @return 回收的块数
 */
uint32_t gc_trigger(gc_manager_t* gc, bool force);

/**
 * @brief 检查是否需要GC
 * @param gc 管理器句柄
 * @return 1需要，0不需要
 */
int gc_check_needed(gc_manager_t* gc);

/**
 * @brief 获取GC统计
 * @param gc 管理器句柄
 * @param stats 统计输出
 */
void gc_get_stats(gc_manager_t* gc, gc_stats_t* stats);

/**
 * @brief 打印GC信息
 * @param gc 管理器句柄
 */
void gc_dump_info(gc_manager_t* gc);

#ifdef __cplusplus
}
#endif

#endif /* __GC_MANAGER_H__ */
