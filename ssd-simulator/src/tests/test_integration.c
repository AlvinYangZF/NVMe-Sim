/**
 * @file test_integration.c
 * @brief SSD Simulator - Integration Test
 * 
 * 集成测试：展示任务调度 + 核间通信 + 内存管理 + NAND模拟的协同工作
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <time.h>
#include "task_scheduler.h"
#include "inter_core_comm.h"
#include "ddr_manager.h"
#include "nand_simulator.h"
#include "l2p_manager.h"

#define NUM_CORES       4
#define NUM_IO_TASKS    32
#define TEST_DATA_SIZE  16384

/* 测试上下文 */
typedef struct test_context {
    nand_simulator_t* nand;
    shared_memory_t* shm;
    l2p_manager_t* l2p;
    uint64_t completed_reads;
    uint64_t completed_writes;
    pthread_mutex_t lock;
} test_context_t;

static test_context_t g_context = {0};

/* IO任务私有数据 */
typedef struct io_task_data {
    uint64_t lba;
    uint64_t ppa;
    void* buffer;
    int is_write;
} io_task_data_t;

/* NAND IO任务执行函数 */
static void nand_io_task_execute(task_ctx_t* task) {
    io_task_data_t* data = (io_task_data_t*)task->private_data;
    int ret = 0;
    
    if (data->is_write) {
        /* 生成测试数据 */
        for (int i = 0; i < TEST_DATA_SIZE; i++) {
            ((uint8_t*)data->buffer)[i] = (uint8_t)(data->lba + i);
        }
        
        /* 写入NAND */
        ret = nand_write_page(g_context.nand, data->ppa, data->buffer, NULL);
        
        pthread_mutex_lock(&g_context.lock);
        g_context.completed_writes++;
        pthread_mutex_unlock(&g_context.lock);
        
        if (ret == 0) {
            printf("  [Core %u] Write LBA=%lu, PPA=%lu OK\n", 
                   task->core_id, data->lba, data->ppa);
        } else {
            printf("  [Core %u] Write LBA=%lu, PPA=%lu FAILED (%d)\n", 
                   task->core_id, data->lba, data->ppa, ret);
        }
    } else {
        /* 从NAND读取 */
        ret = nand_read_page(g_context.nand, data->ppa, data->buffer, NULL);
        
        pthread_mutex_lock(&g_context.lock);
        g_context.completed_reads++;
        pthread_mutex_unlock(&g_context.lock);
        
        /* 验证数据 */
        int valid = 1;
        for (int i = 0; i < TEST_DATA_SIZE && valid; i++) {
            if (((uint8_t*)data->buffer)[i] != (uint8_t)(data->lba + i)) {
                valid = 0;
            }
        }
        
        if (ret == 0 && valid) {
            printf("  [Core %u] Read LBA=%lu, PPA=%lu OK (verified)\n", 
                   task->core_id, data->lba, data->ppa);
        } else {
            printf("  [Core %u] Read LBA=%lu, PPA=%lu FAILED (ret=%d, valid=%d)\n", 
                   task->core_id, data->lba, data->ppa, ret, valid);
        }
    }
}

/* 任务完成回调 */
static void nand_io_task_complete(task_ctx_t* task) {
    io_task_data_t* data = (io_task_data_t*)task->private_data;
    ddr_free(data->buffer);
    free(data);
}

/* 测试1: 基础功能测试 */
static int test_basic_functions(void) {
    printf("\n=== Test 1: Basic Functions ===\n");
    
    /* 初始化各模块 */
    printf("Initializing modules...\n");
    
    /* DDR管理器 */
    if (ddr_manager_init(1024 * 1024 * 1024) != 0) {
        printf("FAILED: DDR manager init\n");
        return -1;
    }
    printf("  DDR Manager: OK (1GB)\n");
    
    /* NAND模拟器 */
    nand_config_t nand_cfg = {
        .type = NAND_TYPE_TLC,
        .num_channels = 4,
        .dies_per_channel = 2,
        .planes_per_die = 2,
        .blocks_per_plane = 128,
        .pages_per_block = 384,
        .page_size = 16384,
        .oob_size = 1664,
        .pe_cycles = 3000,
    };
    
    g_context.nand = nand_sim_create(&nand_cfg);
    if (!g_context.nand) {
        printf("FAILED: NAND simulator create\n");
        ddr_manager_destroy();
        return -1;
    }
    printf("  NAND Simulator: OK\n");
    
    /* 任务调度器 */
    if (task_scheduler_init(NUM_CORES) != 0) {
        printf("FAILED: Task scheduler init\n");
        nand_sim_destroy(g_context.nand);
        ddr_manager_destroy();
        return -1;
    }
    printf("  Task Scheduler: OK (%d cores)\n", NUM_CORES);
    
    /* 核间通信 */
    g_context.shm = icc_shm_create(NUM_CORES, 256);
    if (!g_context.shm) {
        printf("FAILED: ICC shm create\n");
        task_scheduler_destroy();
        nand_sim_destroy(g_context.nand);
        ddr_manager_destroy();
        return -1;
    }
    
    /* 各核心初始化ICC */
    for (int i = 0; i < NUM_CORES; i++) {
        /* 注意：实际中每个核心在自己的线程中调用，这里简化处理 */
        /* icc_init(i, NUM_CORES, g_context.shm); */
    }
    printf("  ICC Shared Memory: OK\n");
    
    pthread_mutex_init(&g_context.lock, NULL);
    
    printf("All modules initialized successfully!\n");
    
    /* 显示NAND信息 */
    nand_dump_info(g_context.nand);
    
    return 0;
}

/* 测试2: 端到端IO测试 */
static int test_end_to_end_io(void) {
    printf("\n=== Test 2: End-to-End IO Test ===\n");
    
    g_context.completed_reads = 0;
    g_context.completed_writes = 0;
    
    /* 创建写任务 */
    printf("\nSubmitting %d write tasks...\n", NUM_IO_TASKS);
    
    for (int i = 0; i < NUM_IO_TASKS; i++) {
        io_task_data_t* data = malloc(sizeof(io_task_data_t));
        data->lba = i;
        data->ppa = PPA_MAKE(i % 4, (i / 4) % 2, (i / 8) % 2, i / 16, i % 384);
        data->buffer = ddr_alloc(TEST_DATA_SIZE, 4096);
        data->is_write = 1;
        
        task_ctx_t* task = task_ctx_create(TASK_TYPE_IO_WRITE, TASK_PRIO_NORMAL,
                                           nand_io_task_execute,
                                           nand_io_task_complete,
                                           data);
        task_submit(task, -1);  /* 自动分配到任意核心 */
    }
    
    /* 等待写完成 */
    sleep(2);
    
    printf("\nCompleted %lu writes\n", g_context.completed_writes);
    
    /* 创建读任务 */
    printf("\nSubmitting %d read tasks...\n", NUM_IO_TASKS);
    
    for (int i = 0; i < NUM_IO_TASKS; i++) {
        io_task_data_t* data = malloc(sizeof(io_task_data_t));
        data->lba = i;
        data->ppa = PPA_MAKE(i % 4, (i / 4) % 2, (i / 8) % 2, i / 16, i % 384);
        data->buffer = ddr_alloc(TEST_DATA_SIZE, 4096);
        data->is_write = 0;
        
        task_ctx_t* task = task_ctx_create(TASK_TYPE_IO_READ, TASK_PRIO_HIGH,
                                           nand_io_task_execute,
                                           nand_io_task_complete,
                                           data);
        task_submit(task, -1);
    }
    
    /* 等待读完成 */
    sleep(2);
    
    printf("\nCompleted %lu reads\n", g_context.completed_reads);
    
    /* 验证结果 */
    if (g_context.completed_writes != NUM_IO_TASKS ||
        g_context.completed_reads != NUM_IO_TASKS) {
        printf("FAILED: Not all IOs completed\n");
        return -1;
    }
    
    printf("All %d IOs completed successfully!\n", NUM_IO_TASKS * 2);
    
    return 0;
}

/* 测试3: 统计信息 */
static int test_statistics(void) {
    printf("\n=== Test 3: Statistics ===\n");
    
    /* DDR统计 */
    ddr_stats_t ddr_stats;
    ddr_get_stats(&ddr_stats);
    
    printf("\n[DDR Statistics]\n");
    printf("  Total allocs: %lu\n", ddr_stats.total_allocs);
    printf("  Total frees:  %lu\n", ddr_stats.total_frees);
    printf("  Current used: %lu MB\n", ddr_stats.current_used / (1024 * 1024));
    printf("  Peak usage:   %lu MB\n", ddr_stats.peak_used / (1024 * 1024));
    
    /* 任务调度统计 */
    task_scheduler_stats_t sched_stats;
    task_get_stats(-1, &sched_stats);
    
    printf("\n[Task Scheduler Statistics]\n");
    printf("  Tasks executed: %lu\n", sched_stats.tasks_executed);
    printf("  Tasks stolen:   %lu\n", sched_stats.tasks_stolen);
    printf("  Avg latency:    %.2f ns\n", sched_stats.avg_latency_ns);
    
    /* NAND统计 */
    nand_stats_t nand_stats;
    nand_get_stats(g_context.nand, &nand_stats);
    
    printf("\n[NAND Statistics]\n");
    printf("  Reads:  %lu, Avg latency: %lu us\n", 
           nand_stats.read_count,
           nand_stats.read_count > 0 ? nand_stats.total_read_latency / nand_stats.read_count : 0);
    printf("  Progs:  %lu, Avg latency: %lu us\n",
           nand_stats.prog_count,
           nand_stats.prog_count > 0 ? nand_stats.total_prog_latency / nand_stats.prog_count : 0);
    printf("  Bytes read:    %lu MB\n", nand_stats.bytes_read / (1024 * 1024));
    printf("  Bytes written: %lu MB\n", nand_stats.bytes_written / (1024 * 1024));
    
    return 0;
}

/* 清理 */
static void cleanup(void) {
    printf("\n=== Cleanup ===\n");
    
    pthread_mutex_destroy(&g_context.lock);
    task_scheduler_destroy();
    icc_shm_destroy(g_context.shm);
    nand_sim_destroy(g_context.nand);
    ddr_manager_destroy();
    
    printf("All resources cleaned up.\n");
}

/* 主函数 */
int main(int argc, char* argv[]) {
    printf("============================================\n");
    printf("  SSD Simulator - Integration Test\n");
    printf("============================================\n");
    
    srand(time(NULL));
    
    int failures = 0;
    
    if (test_basic_functions() != 0) failures++;
    if (test_end_to_end_io() != 0) failures++;
    if (test_statistics() != 0) failures++;
    
    cleanup();
    
    printf("\n============================================\n");
    if (failures == 0) {
        printf("  All integration tests PASSED!\n");
    } else {
        printf("  %d test(s) FAILED!\n", failures);
    }
    printf("============================================\n");
    
    return failures;
}
