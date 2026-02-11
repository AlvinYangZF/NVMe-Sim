/**
 * @file nand_simulator.h
 * @brief SSD Simulator - NAND Simulator Header
 */

#ifndef __NAND_SIMULATOR_H__
#define __NAND_SIMULATOR_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* NAND类型 */
typedef enum {
    NAND_TYPE_SLC = 0,
    NAND_TYPE_MLC,
    NAND_TYPE_TLC,
    NAND_TYPE_QLC,
    NAND_TYPE_NUM
} nand_type_t;

/* 页状态 */
typedef enum {
    PAGE_STATE_FREE = 0,
    PAGE_STATE_VALID,
    PAGE_STATE_INVALID,
    PAGE_STATE_WRITING,
} page_state_t;

/* 块状态 */
typedef enum {
    BLOCK_STATE_FREE = 0,
    BLOCK_STATE_OPEN,
    BLOCK_STATE_FULL,
    BLOCK_STATE_GC,
    BLOCK_STATE_BAD,
} block_state_t;

/* NAND时序 (微秒) */
typedef struct nand_timing {
    uint32_t t_read;            /* 页读取时间 */
    uint32_t t_prog;            /* 页编程时间 */
    uint32_t t_erase;           /* 块擦除时间 */
    uint32_t t_transfer;        /* 数据传输时间 */
} nand_timing_t;

/* NAND配置 */
typedef struct nand_config {
    nand_type_t type;
    
    /* 几何参数 */
    uint32_t num_channels;
    uint32_t dies_per_channel;
    uint32_t planes_per_die;
    uint32_t blocks_per_plane;
    uint32_t pages_per_block;
    
    /* 页大小 */
    uint32_t page_size;         /* 数据区大小 */
    uint32_t oob_size;          /* OOB区大小 */
    
    /* 时序 */
    nand_timing_t timing;
    
    /* 可靠性 */
    uint32_t pe_cycles;         /* 擦写寿命 */
} nand_config_t;

/* PPA格式 */
#define PPA_CHANNEL_BITS    3
#define PPA_DIE_BITS        2
#define PPA_PLANE_BITS      1
#define PPA_BLOCK_BITS      10
#define PPA_PAGE_BITS       9

#define PPA_CHANNEL_MASK    ((1ULL << PPA_CHANNEL_BITS) - 1)
#define PPA_DIE_MASK        ((1ULL << PPA_DIE_BITS) - 1)
#define PPA_PLANE_MASK      ((1ULL << PPA_PLANE_BITS) - 1)
#define PPA_BLOCK_MASK      ((1ULL << PPA_BLOCK_BITS) - 1)
#define PPA_PAGE_MASK       ((1ULL << PPA_PAGE_BITS) - 1)

#define PPA_GET_CHANNEL(ppa)    (((ppa) >> 0)  & PPA_CHANNEL_MASK)
#define PPA_GET_DIE(ppa)        (((ppa) >> 3)  & PPA_DIE_MASK)
#define PPA_GET_PLANE(ppa)      (((ppa) >> 5)  & PPA_PLANE_MASK)
#define PPA_GET_BLOCK(ppa)      (((ppa) >> 6)  & PPA_BLOCK_MASK)
#define PPA_GET_PAGE(ppa)       (((ppa) >> 16) & PPA_PAGE_MASK)

#define PPA_MAKE(ch, die, pl, blk, pg) \
    (((uint64_t)(ch)  << 0)  | \
     ((uint64_t)(die) << 3)  | \
     ((uint64_t)(pl)  << 5)  | \
     ((uint64_t)(blk) << 6)  | \
     ((uint64_t)(pg)  << 16))

#define INVALID_PPA     (~0ULL)

/* NAND统计 */
typedef struct nand_stats {
    uint64_t read_count;
    uint64_t prog_count;
    uint64_t erase_count;
    uint64_t read_error_count;
    uint64_t prog_error_count;
    uint64_t erase_error_count;
    
    uint64_t total_read_latency;
    uint64_t total_prog_latency;
    uint64_t total_erase_latency;
    
    uint64_t bytes_read;
    uint64_t bytes_written;
} nand_stats_t;

/* Opaque类型 */
typedef struct nand_simulator nand_simulator_t;

/**
 * @brief 初始化NAND模拟器
 * @param config NAND配置
 * @return 模拟器句柄，NULL失败
 */
nand_simulator_t* nand_sim_create(nand_config_t* config);

/**
 * @brief 销毁NAND模拟器
 * @param sim 模拟器句柄
 */
void nand_sim_destroy(nand_simulator_t* sim);

/**
 * @brief 读取页
 * @param sim 模拟器句柄
 * @param ppa 物理页地址
 * @param data_buf 数据缓冲区
 * @param oob_buf OOB缓冲区 (可为NULL)
 * @return 0成功，负数错误码
 */
int nand_read_page(nand_simulator_t* sim, uint64_t ppa, void* data_buf, void* oob_buf);

/**
 * @brief 编程页
 * @param sim 模拟器句柄
 * @param ppa 物理页地址
 * @param data_buf 数据缓冲区
 * @param oob_buf OOB缓冲区 (可为NULL)
 * @return 0成功，负数错误码
 */
int nand_write_page(nand_simulator_t* sim, uint64_t ppa, void* data_buf, void* oob_buf);

/**
 * @brief 擦除块
 * @param sim 模拟器句柄
 * @param ppa 物理页地址 (块内任意页)
 * @return 0成功，负数错误码
 */
int nand_erase_block(nand_simulator_t* sim, uint64_t ppa);

/**
 * @brief 获取页状态
 * @param sim 模拟器句柄
 * @param ppa 物理页地址
 * @return 页状态
 */
page_state_t nand_get_page_state(nand_simulator_t* sim, uint64_t ppa);

/**
 * @brief 获取块状态
 * @param sim 模拟器句柄
 * @param ppa 物理页地址
 * @return 块状态
 */
block_state_t nand_get_block_state(nand_simulator_t* sim, uint64_t ppa);

/**
 * @brief 获取块擦除次数
 * @param sim 模拟器句柄
 * @param ppa 物理页地址
 * @return 擦除次数
 */
uint32_t nand_get_erase_count(nand_simulator_t* sim, uint64_t ppa);

/**
 * @brief 标记坏块
 * @param sim 模拟器句柄
 * @param ppa 物理页地址
 * @return 0成功
 */
int nand_mark_bad_block(nand_simulator_t* sim, uint64_t ppa);

/**
 * @brief 检查是否为坏块
 * @param sim 模拟器句柄
 * @param ppa 物理页地址
 * @return 1是坏块，0不是
 */
int nand_is_bad_block(nand_simulator_t* sim, uint64_t ppa);

/**
 * @brief 设置错误注入
 * @param sim 模拟器句柄
 * @param error_rate 错误率 (0.0 - 1.0)
 */
void nand_set_error_rate(nand_simulator_t* sim, double error_rate);

/**
 * @brief 获取统计信息
 * @param sim 模拟器句柄
 * @param stats 统计输出
 */
void nand_get_stats(nand_simulator_t* sim, nand_stats_t* stats);

/**
 * @brief 打印NAND信息
 * @param sim 模拟器句柄
 */
void nand_dump_info(nand_simulator_t* sim);

#ifdef __cplusplus
}
#endif

#endif /* __NAND_SIMULATOR_H__ */
