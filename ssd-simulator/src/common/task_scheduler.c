/**
 * @file task_scheduler.c
 * @brief SSD Simulator - Task Scheduler Implementation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "task_scheduler.h"

/* 调度器实例 */
typedef struct task_scheduler {
    uint32_t core_id;
    task_queue_t local_queue;
    
    struct task_scheduler** all_scheds;
    uint32_t num_cores;
    uint32_t steal_attempts;
    
    pthread_t worker_thread;
    volatile int running;
    
    uint64_t tasks_executed;
    uint64_t tasks_stolen;
    uint64_t total_latency_ns;
} task_scheduler_t;

/* 全局调度器 */
typedef struct global_scheduler {
    task_scheduler_t** schedulers;
    uint32_t num_cores;
    uint64_t next_task_id;
    pthread_mutex_t id_lock;
    uint32_t rr_counter;
    pthread_mutex_t rr_lock;
    int initialized;
} global_scheduler_t;

/* 全局实例 */
static global_scheduler_t g_sched = {0};

/* 线程局部存储 */
static __thread task_scheduler_t* tls_scheduler = NULL;

/* 前向声明 */
static void* worker_thread(void* arg);
static task_ctx_t* task_schedule(task_scheduler_t* sched);
static task_ctx_t* task_steal(task_scheduler_t* sched);

int task_scheduler_init(uint32_t num_cores) {
    if (g_sched.initialized) {
        return -EALREADY;
    }
    
    if (num_cores == 0 || num_cores > 128) {
        return -EINVAL;
    }
    
    g_sched.num_cores = num_cores;
    g_sched.next_task_id = 1;
    pthread_mutex_init(&g_sched.id_lock, NULL);
    pthread_mutex_init(&g_sched.rr_lock, NULL);
    g_sched.rr_counter = 0;
    
    /* 分配调度器数组 */
    g_sched.schedulers = calloc(num_cores, sizeof(task_scheduler_t*));
    if (!g_sched.schedulers) {
        return -ENOMEM;
    }
    
    /* 初始化每个核心的调度器 */
    for (uint32_t i = 0; i < num_cores; i++) {
        task_scheduler_t* sched = calloc(1, sizeof(task_scheduler_t));
        if (!sched) {
            goto err_cleanup;
        }
        
        sched->core_id = i;
        sched->num_cores = num_cores;
        sched->all_scheds = g_sched.schedulers;
        sched->running = 1;
        
        /* 初始化队列 */
        pthread_spin_init(&sched->local_queue.lock, PTHREAD_PROCESS_PRIVATE);
        
        g_sched.schedulers[i] = sched;
    }
    
    /* 启动工作线程 */
    for (uint32_t i = 0; i < num_cores; i++) {
        if (pthread_create(&g_sched.schedulers[i]->worker_thread, NULL, 
                           worker_thread, g_sched.schedulers[i]) != 0) {
            goto err_cleanup;
        }
    }
    
    g_sched.initialized = 1;
    printf("[TaskScheduler] Initialized with %u cores\n", num_cores);
    return 0;

err_cleanup:
    for (uint32_t i = 0; i < num_cores; i++) {
        if (g_sched.schedulers[i]) {
            g_sched.schedulers[i]->running = 0;
            if (g_sched.schedulers[i]->worker_thread) {
                pthread_join(g_sched.schedulers[i]->worker_thread, NULL);
            }
            free(g_sched.schedulers[i]);
        }
    }
    free(g_sched.schedulers);
    memset(&g_sched, 0, sizeof(g_sched));
    return -ENOMEM;
}

void task_scheduler_destroy(void) {
    if (!g_sched.initialized) {
        return;
    }
    
    /* 停止所有工作线程 */
    for (uint32_t i = 0; i < g_sched.num_cores; i++) {
        if (g_sched.schedulers[i]) {
            g_sched.schedulers[i]->running = 0;
        }
    }
    
    /* 等待线程结束 */
    for (uint32_t i = 0; i < g_sched.num_cores; i++) {
        if (g_sched.schedulers[i] && g_sched.schedulers[i]->worker_thread) {
            pthread_join(g_sched.schedulers[i]->worker_thread, NULL);
        }
    }
    
    /* 清理资源 */
    for (uint32_t i = 0; i < g_sched.num_cores; i++) {
        if (g_sched.schedulers[i]) {
            pthread_spin_destroy(&g_sched.schedulers[i]->local_queue.lock);
            free(g_sched.schedulers[i]);
        }
    }
    
    pthread_mutex_destroy(&g_sched.id_lock);
    pthread_mutex_destroy(&g_sched.rr_lock);
    free(g_sched.schedulers);
    
    memset(&g_sched, 0, sizeof(g_sched));
    printf("[TaskScheduler] Destroyed\n");
}

int task_submit(task_ctx_t* task, int32_t preferred_core) {
    if (!g_sched.initialized || !task) {
        return -EINVAL;
    }
    
    /* 分配任务ID */
    pthread_mutex_lock(&g_sched.id_lock);
    task->task_id = g_sched.next_task_id++;
    pthread_mutex_unlock(&g_sched.id_lock);
    
    task->submit_time = get_time_ns();
    task->state = TASK_STATE_PENDING;
    
    /* 选择目标核心 */
    uint32_t target_core;
    if (preferred_core >= 0 && (uint32_t)preferred_core < g_sched.num_cores) {
        target_core = preferred_core;
    } else {
        pthread_mutex_lock(&g_sched.rr_lock);
        target_core = g_sched.rr_counter++ % g_sched.num_cores;
        pthread_mutex_unlock(&g_sched.rr_lock);
    }
    
    task->core_id = target_core;
    
    /* 加入目标核心队列 */
    task_scheduler_t* sched = g_sched.schedulers[target_core];
    task_queue_t* queue = &sched->local_queue;
    
    pthread_spin_lock(&queue->lock);
    
    task->next = NULL;
    task->prev = queue->tails[task->priority];
    
    if (queue->tails[task->priority]) {
        queue->tails[task->priority]->next = task;
    } else {
        queue->heads[task->priority] = task;
    }
    queue->tails[task->priority] = task;
    queue->counts[task->priority]++;
    queue->total_count++;
    
    pthread_spin_unlock(&queue->lock);
    
    return 0;
}

int task_submit_batch(task_ctx_t** tasks, uint32_t count, int32_t preferred_core) {
    if (!tasks || count == 0) {
        return 0;
    }
    
    int submitted = 0;
    for (uint32_t i = 0; i < count; i++) {
        if (task_submit(tasks[i], preferred_core) == 0) {
            submitted++;
        }
    }
    
    return submitted;
}

int task_cancel(uint64_t task_id) {
    /* 遍历所有队列查找任务 */
    for (uint32_t c = 0; c < g_sched.num_cores; c++) {
        task_scheduler_t* sched = g_sched.schedulers[c];
        task_queue_t* queue = &sched->local_queue;
        
        pthread_spin_lock(&queue->lock);
        
        for (int prio = 0; prio < TASK_PRIO_NUM; prio++) {
            task_ctx_t* task = queue->heads[prio];
            while (task) {
                if (task->task_id == task_id) {
                    /* 从队列中移除 */
                    if (task->prev) {
                        task->prev->next = task->next;
                    } else {
                        queue->heads[prio] = task->next;
                    }
                    if (task->next) {
                        task->next->prev = task->prev;
                    } else {
                        queue->tails[prio] = task->prev;
                    }
                    queue->counts[prio]--;
                    queue->total_count--;
                    
                    task->state = TASK_STATE_CANCELLED;
                    pthread_spin_unlock(&queue->lock);
                    return 0;
                }
                task = task->next;
            }
        }
        
        pthread_spin_unlock(&queue->lock);
    }
    
    return -ENOENT;
}

uint32_t task_queue_depth(int32_t core_id) {
    if (!g_sched.initialized) {
        return 0;
    }
    
    if (core_id < 0) {
        /* 总计 */
        uint32_t total = 0;
        for (uint32_t i = 0; i < g_sched.num_cores; i++) {
            total += g_sched.schedulers[i]->local_queue.total_count;
        }
        return total;
    }
    
    if ((uint32_t)core_id >= g_sched.num_cores) {
        return 0;
    }
    
    return g_sched.schedulers[core_id]->local_queue.total_count;
}

int task_get_stats(int32_t core_id, task_scheduler_stats_t* stats) {
    if (!g_sched.initialized || !stats) {
        return -EINVAL;
    }
    
    memset(stats, 0, sizeof(*stats));
    
    if (core_id < 0) {
        /* 总计 */
        for (uint32_t i = 0; i < g_sched.num_cores; i++) {
            task_scheduler_t* sched = g_sched.schedulers[i];
            stats->tasks_executed += sched->tasks_executed;
            stats->tasks_stolen += sched->tasks_stolen;
            stats->total_latency_ns += sched->total_latency_ns;
            
            for (int p = 0; p < TASK_PRIO_NUM; p++) {
                stats->queue_depth[p] += sched->local_queue.counts[p];
            }
        }
        if (stats->tasks_executed > 0) {
            stats->avg_latency_ns = (double)stats->total_latency_ns / stats->tasks_executed;
        }
    } else {
        if ((uint32_t)core_id >= g_sched.num_cores) {
            return -EINVAL;
        }
        
        task_scheduler_t* sched = g_sched.schedulers[core_id];
        stats->tasks_executed = sched->tasks_executed;
        stats->tasks_stolen = sched->tasks_stolen;
        stats->total_latency_ns = sched->total_latency_ns;
        if (stats->tasks_executed > 0) {
            stats->avg_latency_ns = (double)stats->total_latency_ns / stats->tasks_executed;
        }
        for (int p = 0; p < TASK_PRIO_NUM; p++) {
            stats->queue_depth[p] = sched->local_queue.counts[p];
        }
    }
    
    return 0;
}

task_ctx_t* task_ctx_create(task_type_t type, task_prio_t priority,
                            task_execute_fn_t execute,
                            task_complete_fn_t complete,
                            void* private_data) {
    task_ctx_t* task = calloc(1, sizeof(task_ctx_t));
    if (!task) {
        return NULL;
    }
    
    task->type = type;
    task->priority = priority;
    task->execute = execute;
    task->complete = complete;
    task->private_data = private_data;
    task->state = TASK_STATE_PENDING;
    
    return task;
}

void task_ctx_free(task_ctx_t* task) {
    free(task);
}

/* 工作线程 */
static void* worker_thread(void* arg) {
    task_scheduler_t* sched = (task_scheduler_t*)arg;
    tls_scheduler = sched;
    
    printf("[TaskScheduler] Core %u worker started\n", sched->core_id);
    
    while (sched->running) {
        task_ctx_t* task = task_schedule(sched);
        
        if (task) {
            /* 执行任务 */
            task->state = TASK_STATE_RUNNING;
            task->start_time = get_time_ns();
            
            if (task->execute) {
                task->execute(task);
            }
            
            /* 更新统计 */
            task->complete_time = get_time_ns();
            task->state = TASK_STATE_COMPLETED;
            
            sched->tasks_executed++;
            sched->total_latency_ns += (task->complete_time - task->submit_time);
            
            /* 完成回调 */
            if (task->complete) {
                task->complete(task);
            }
        } else {
            /* 无任务，短暂休眠 */
            cpu_relax();
            struct timespec ts = {0, 1000}; /* 1us */
            nanosleep(&ts, NULL);
        }
    }
    
    printf("[TaskScheduler] Core %u worker stopped\n", sched->core_id);
    return NULL;
}

/* 任务调度 */
static task_ctx_t* task_schedule(task_scheduler_t* sched) {
    task_queue_t* queue = &sched->local_queue;
    task_ctx_t* task = NULL;
    
    /* 1. 尝试从本地队列获取任务 (优先级从高到低) */
    pthread_spin_lock(&queue->lock);
    
    for (int prio = TASK_PRIO_HIGHEST; prio < TASK_PRIO_NUM; prio++) {
        if (queue->heads[prio] != NULL) {
            task = queue->heads[prio];
            queue->heads[prio] = task->next;
            if (queue->heads[prio]) {
                queue->heads[prio]->prev = NULL;
            } else {
                queue->tails[prio] = NULL;
            }
            queue->counts[prio]--;
            queue->total_count--;
            
            task->next = NULL;
            task->prev = NULL;
            break;
        }
    }
    
    pthread_spin_unlock(&queue->lock);
    
    /* 2. 本地无任务，尝试Work Stealing */
    if (task == NULL) {
        task = task_steal(sched);
    }
    
    return task;
}

/* Work Stealing */
static task_ctx_t* task_steal(task_scheduler_t* sched) {
    if (sched->num_cores <= 1) {
        return NULL;
    }
    
    /* 随机选择 victim core */
    uint32_t victim = (sched->core_id + 1 + rand() % (sched->num_cores - 1)) % sched->num_cores;
    task_scheduler_t* victim_sched = sched->all_scheds[victim];
    task_queue_t* victim_queue = &victim_sched->local_queue;
    
    task_ctx_t* task = NULL;
    
    /* 尝试获取锁 */
    if (pthread_spin_trylock(&victim_queue->lock) != 0) {
        return NULL;
    }
    
    /* 从victim的最低优先级队列偷取 (减少干扰) */
    for (int prio = TASK_PRIO_LOWEST; prio >= TASK_PRIO_NORMAL; prio--) {
        /* 保留至少一个任务 */
        if (victim_queue->counts[prio] > 1) {
            /* 从队尾偷取 */
            task = victim_queue->tails[prio];
            if (task) {
                victim_queue->tails[prio] = task->prev;
                if (victim_queue->tails[prio]) {
                    victim_queue->tails[prio]->next = NULL;
                } else {
                    victim_queue->heads[prio] = NULL;
                }
                victim_queue->counts[prio]--;
                victim_queue->total_count--;
                
                task->next = NULL;
                task->prev = NULL;
                task->src_core = victim;
                
                sched->tasks_stolen++;
                break;
            }
        }
    }
    
    pthread_spin_unlock(&victim_queue->lock);
    
    return task;
}
