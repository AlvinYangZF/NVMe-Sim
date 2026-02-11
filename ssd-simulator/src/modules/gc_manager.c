/**
 * @file gc_manager.c
 * @brief SSD Simulator - Garbage Collection Manager Implementation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include "gc_manager.h"
#include "ddr_manager.h"

/* 块信息 */
typedef struct block_info {
    uint64_t ppa;               /* 块的起始PPA */
    uint32_t valid_pages;       /* 有效页数 */
    uint32_t invalid_pages;     /* 无效页数 */
    uint32_t erase_count;       /* 擦除次数 */
    double   gc_cost;           /* GC成本 */
} block_info_t;

/* GC管理器 */
struct gc_manager {
    nand_simulator_t* nand;
    l2p_manager_t* l2p;
    gc_config_t config;
    
    /* 候选块列表 */
    block_info_t* victim_list;
    uint32_t victim_count;
    uint32_t victim_capacity;
    
    /* 统计 */
    gc_stats_t stats;
    pthread_mutex_t stats_lock;
    
    /* 后台线程 */
    pthread_t bg_thread;
    volatile int running;
};

/* 前向声明 */
static void* gc_background_thread(void* arg);
static int select_victim_blocks(gc_manager_t* gc);
static int migrate_valid_pages(gc_manager_t* gc, block_info_t* block);
static double calculate_gc_cost(gc_manager_t* gc, block_info_t* block);
static inline uint64_t get_time_ms(void);

gc_manager_t* gc_manager_create(nand_simulator_t* nand, 
                                 l2p_manager_t* l2p,
                                 gc_config_t* config) {
    if (!nand || !l2p || !config) return NULL;
    
    gc_manager_t* gc = calloc(1, sizeof(gc_manager_t));
    if (!gc) return NULL;
    
    gc->nand = nand;
    gc->l2p = l2p;
    memcpy(&gc->config, config, sizeof(gc_config_t));
    
    /* 分配候选块列表 */
    gc->victim_capacity = config->max_victim_blocks;
    gc->victim_list = calloc(gc->victim_capacity, sizeof(block_info_t));
    if (!gc->victim_list) {
        free(gc);
        return NULL;
    }
    
    pthread_mutex_init(&gc->stats_lock, NULL);
    
    /* 启动后台线程 */
    if (config->enable_background) {
        gc->running = 1;
        pthread_create(&gc->bg_thread, NULL, gc_background_thread, gc);
    }
    
    printf("[GC] Manager created: policy=%s, threshold=%u%%\n",
           config->policy == GC_POLICY_GREEDY ? "greedy" :
           config->policy == GC_POLICY_COST_BENEFIT ? "cost-benefit" : "wear-aware",
           config->threshold);
    
    return gc;
}

void gc_manager_destroy(gc_manager_t* gc) {
    if (!gc) return;
    
    /* 停止后台线程 */
    if (gc->config.enable_background) {
        gc->running = 0;
        pthread_join(gc->bg_thread, NULL);
    }
    
    pthread_mutex_destroy(&gc->stats_lock);
    free(gc->victim_list);
    free(gc);
    
    printf("[GC] Manager destroyed\n");
}

uint32_t gc_trigger(gc_manager_t* gc, bool force) {
    if (!gc) return 0;
    
    uint64_t start_time = get_time_ms();
    
    /* 选择候选块 */
    uint32_t victim_count = select_victim_blocks(gc);
    if (victim_count == 0) {
        return 0;
    }
    
    printf("[GC] Triggered: %u victim blocks selected\n", victim_count);
    
    /* 迁移有效页 */
    uint32_t migrated_pages = 0;
    uint32_t reclaimed_blocks = 0;
    
    for (uint32_t i = 0; i < victim_count; i++) {
        block_info_t* block = &gc->victim_list[i];
        
        if (block->valid_pages > 0) {
            /* 迁移有效页 */
            int ret = migrate_valid_pages(gc, block);
            if (ret > 0) {
                migrated_pages += ret;
            }
        }
        
        /* 擦除块 */
        nand_erase_block(gc->nand, block->ppa);
        reclaimed_blocks++;
    }
    
    uint64_t gc_time = get_time_ms() - start_time;
    
    /* 更新统计 */
    pthread_mutex_lock(&gc->stats_lock);
    gc->stats.gc_count++;
    gc->stats.blocks_reclaimed += reclaimed_blocks;
    gc->stats.pages_migrated += migrated_pages;
    gc->stats.bytes_migrated += (uint64_t)migrated_pages * 16384;
    gc->stats.total_gc_time_ms += gc_time;
    gc->stats.avg_gc_time_ms = 
        (double)gc->stats.total_gc_time_ms / gc->stats.gc_count;
    pthread_mutex_unlock(&gc->stats_lock);
    
    printf("[GC] Completed: %u blocks reclaimed, %u pages migrated, %lu ms\n",
           reclaimed_blocks, migrated_pages, gc_time);
    
    return reclaimed_blocks;
}

int gc_check_needed(gc_manager_t* gc) {
    if (!gc) return 0;
    
    /* 简化实现：总是检查NAND状态 */
    /* 实际实现会统计无效页比例 */
    
    /* 选择候选块 */
    uint32_t victim_count = select_victim_blocks(gc);
    
    pthread_mutex_lock(&gc->stats_lock);
    gc->stats.current_victim_blocks = victim_count;
    pthread_mutex_unlock(&gc->stats_lock);
    
    return victim_count > 0;
}

void gc_get_stats(gc_manager_t* gc, gc_stats_t* stats) {
    if (!gc || !stats) return;
    
    pthread_mutex_lock(&gc->stats_lock);
    memcpy(stats, &gc->stats, sizeof(gc_stats_t));
    pthread_mutex_unlock(&gc->stats_lock);
}

void gc_dump_info(gc_manager_t* gc) {
    if (!gc) return;
    
    gc_stats_t stats;
    gc_get_stats(gc, &gc->stats);
    
    printf("\n[GC] Manager Info:\n");
    printf("  Policy: %s\n",
           gc->config.policy == GC_POLICY_GREEDY ? "greedy" :
           gc->config.policy == GC_POLICY_COST_BENEFIT ? "cost-benefit" : "wear-aware");
    printf("  Threshold: %u%%\n", gc->config.threshold);
    printf("  Background: %s\n", gc->config.enable_background ? "enabled" : "disabled");
    
    printf("\n  Statistics:\n");
    printf("    GC count: %lu\n", stats.gc_count);
    printf("    Blocks reclaimed: %lu\n", stats.blocks_reclaimed);
    printf("    Pages migrated: %lu\n", stats.pages_migrated);
    printf("    Data migrated: %lu MB\n", stats.bytes_migrated / (1024 * 1024));
    printf("    Avg GC time: %.2f ms\n", stats.avg_gc_time_ms);
    printf("    Current victim blocks: %u\n", stats.current_victim_blocks);
}

/* 后台线程 */
static void* gc_background_thread(void* arg) {
    gc_manager_t* gc = (gc_manager_t*)arg;
    
    printf("[GC] Background thread started\n");
    
    while (gc->running) {
        /* 检查是否需要GC */
        if (gc_check_needed(gc)) {
            gc_trigger(gc, false);
        }
        
        /* 休眠1秒 */
        sleep(1);
    }
    
    printf("[GC] Background thread stopped\n");
    return NULL;
}

/* 选择候选块 */
static int select_victim_blocks(gc_manager_t* gc) {
    /* 简化实现：扫描NAND选择无效页最多的块 */
    /* 实际实现会维护一个候选块列表 */
    
    gc->victim_count = 0;
    
    /* 模拟：随机选择几个块作为候选 */
    uint32_t num_victims = 4;
    if (num_victims > gc->victim_capacity) {
        num_victims = gc->victim_capacity;
    }
    
    for (uint32_t i = 0; i < num_victims; i++) {
        /* 生成测试PPA */
        uint64_t ppa = PPA_MAKE(i % 4, (i / 4) % 2, 0, i / 8, 0);
        
        gc->victim_list[i].ppa = ppa;
        gc->victim_list[i].valid_pages = rand() % 100;
        gc->victim_list[i].invalid_pages = 384 - gc->victim_list[i].valid_pages;
        gc->victim_list[i].erase_count = rand() % 1000;
        gc->victim_list[i].gc_cost = calculate_gc_cost(gc, &gc->victim_list[i]);
        
        gc->victim_count++;
    }
    
    return gc->victim_count;
}

/* 迁移有效页 */
static int migrate_valid_pages(gc_manager_t* gc, block_info_t* block) {
    /* 简化实现：仅返回有效页数 */
    /* 实际实现会读取有效页、分配新PPA、写入新位置、更新L2P */
    
    return block->valid_pages;
}

/* 计算GC成本 */
static double calculate_gc_cost(gc_manager_t* gc, block_info_t* block) {
    if (gc->config.policy == GC_POLICY_GREEDY) {
        /* 贪心：无效页越多成本越低 */
        return 1.0 / (block->invalid_pages + 1);
    } else if (gc->config.policy == GC_POLICY_COST_BENEFIT) {
        /* 成本收益：考虑擦除时间和有效页数 */
        return (double)block->valid_pages / (block->invalid_pages + 1);
    } else {
        /* 磨损感知：考虑擦除次数 */
        return (double)block->erase_count / (block->invalid_pages + 1);
    }
}

static inline uint64_t get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}
