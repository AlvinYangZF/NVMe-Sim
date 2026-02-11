/**
 * @file task_scheduler.h
 * @brief SSD Simulator - Task Scheduler Header
 * 
 * 多核任务调度器，支持Work Stealing算法
 */

#ifndef __TASK_SCHEDULER_H__
#define __TASK_SCHEDULER_H__

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 任务类型 */
typedef enum {
    TASK_TYPE_IO_READ = 0,
    TASK_TYPE_IO_WRITE,
    TASK_TYPE_IO_FLUSH,
    TASK_TYPE_GC,
    TASK_TYPE_WL,
    TASK_TYPE_ADMIN,
    TASK_TYPE_BG,
    TASK_TYPE_NUM
} task_type_t;

/* 任务优先级 */
typedef enum {
    TASK_PRIO_HIGHEST = 0,
    TASK_PRIO_HIGH,
    TASK_PRIO_NORMAL,
    TASK_PRIO_LOW,
    TASK_PRIO_LOWEST,
    TASK_PRIO_NUM
} task_prio_t;

/* 任务状态 */
typedef enum {
    TASK_STATE_PENDING = 0,
    TASK_STATE_RUNNING,
    TASK_STATE_BLOCKED,
    TASK_STATE_COMPLETED,
    TASK_STATE_CANCELLED
} task_state_t;

/* 前向声明 */
struct task_ctx;

/* 任务执行回调 */
typedef void (*task_execute_fn_t)(struct task_ctx* task);
typedef void (*task_complete_fn_t)(struct task_ctx* task);

/* 任务上下文 */
typedef struct task_ctx {
    uint64_t task_id;
    task_type_t type;
    task_prio_t priority;
    task_state_t state;
    
    uint32_t core_id;
    uint32_t src_core;
    
    task_execute_fn_t execute;
    task_complete_fn_t complete;
    void* private_data;
    
    uint64_t submit_time;
    uint64_t start_time;
    uint64_t complete_time;
    
    struct task_ctx* next;
    struct task_ctx* prev;
} task_ctx_t;

/* 任务队列 */
typedef struct task_queue {
    task_ctx_t* heads[TASK_PRIO_NUM];
    task_ctx_t* tails[TASK_PRIO_NUM];
    uint32_t counts[TASK_PRIO_NUM];
    uint32_t total_count;
    
    pthread_spinlock_t lock;
} task_queue_t;

/* 调度器统计 */
typedef struct task_scheduler_stats {
    uint64_t tasks_executed;
    uint64_t tasks_stolen;
    uint64_t total_latency_ns;
    double avg_latency_ns;
    uint32_t queue_depth[TASK_PRIO_NUM];
} task_scheduler_stats_t;

/* 调度器实例 (opaque) */
typedef struct task_scheduler task_scheduler_t;

/* 全局调度器 (opaque) */
typedef struct global_scheduler global_scheduler_t;

/* API函数 */

/**
 * @brief 初始化全局任务调度器
 * @param num_cores 核心数
 * @return 0成功，负数错误码
 */
int task_scheduler_init(uint32_t num_cores);

/**
 * @brief 销毁全局任务调度器
 */
void task_scheduler_destroy(void);

/**
 * @brief 提交任务
 * @param task 任务上下文
 * @param preferred_core 首选核心(-1表示自动分配)
 * @return 0成功，负数错误码
 */
int task_submit(task_ctx_t* task, int32_t preferred_core);

/**
 * @brief 批量提交任务
 * @param tasks 任务数组
 * @param count 任务数
 * @param preferred_core 首选核心
 * @return 成功提交的任务数
 */
int task_submit_batch(task_ctx_t** tasks, uint32_t count, int32_t preferred_core);

/**
 * @brief 取消任务
 * @param task_id 任务ID
 * @return 0成功，负数错误码
 */
int task_cancel(uint64_t task_id);

/**
 * @brief 获取队列深度
 * @param core_id 核心ID(-1表示总计)
 * @return 队列深度
 */
uint32_t task_queue_depth(int32_t core_id);

/**
 * @brief 获取调度统计
 * @param core_id 核心ID(-1表示总计)
 * @param stats 统计信息输出
 * @return 0成功，负数错误码
 */
int task_get_stats(int32_t core_id, task_scheduler_stats_t* stats);

/**
 * @brief 创建任务上下文
 * @param type 任务类型
 * @param priority 优先级
 * @param execute 执行函数
 * @param complete 完成回调
 * @param private_data 私有数据
 * @return 任务上下文，NULL失败
 */
task_ctx_t* task_ctx_create(task_type_t type, task_prio_t priority,
                            task_execute_fn_t execute,
                            task_complete_fn_t complete,
                            void* private_data);

/**
 * @brief 释放任务上下文
 * @param task 任务上下文
 */
void task_ctx_free(task_ctx_t* task);

/* 辅助函数 */
#define cpu_relax() __asm__ volatile("pause" ::: "memory")
static inline uint64_t get_time_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

#ifdef __cplusplus
}
#endif

#endif /* __TASK_SCHEDULER_H__ */
