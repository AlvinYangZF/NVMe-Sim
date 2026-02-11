/**
 * @file test_task_scheduler.c
 * @brief Unit Tests for Task Scheduler
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include "task_scheduler.h"

#define TEST_ASSERT(cond) do { \
    if (!(cond)) { \
        printf("FAILED: %s at line %d\n", #cond, __LINE__); \
        return -1; \
    } \
} while(0)

#define TEST_PASS(name) printf("PASSED: %s\n", name)

/* 测试计数 */
static volatile uint64_t g_tasks_completed = 0;
static pthread_mutex_t g_count_lock = PTHREAD_MUTEX_INITIALIZER;

/* 任务执行函数 */
static void test_task_execute(task_ctx_t* task) {
    /* 模拟工作负载 */
    usleep(100); /* 100us */
    
    pthread_mutex_lock(&g_count_lock);
    g_tasks_completed++;
    pthread_mutex_unlock(&g_count_lock);
}

/* 测试1: 基本初始化和销毁 */
static int test_init_destroy(void) {
    printf("\n=== Test: Init/Destroy ===\n");
    
    /* 正常初始化 */
    TEST_ASSERT(task_scheduler_init(4) == 0);
    task_scheduler_destroy();
    
    /* 重复初始化应失败 */
    TEST_ASSERT(task_scheduler_init(4) == 0);
    TEST_ASSERT(task_scheduler_init(4) == -EALREADY);
    task_scheduler_destroy();
    
    /* 边界条件 */
    TEST_ASSERT(task_scheduler_init(0) == -EINVAL);
    TEST_ASSERT(task_scheduler_init(129) == -EINVAL);
    
    TEST_PASS("Init/Destroy");
    return 0;
}

/* 测试2: 单任务提交和执行 */
static int test_single_task(void) {
    printf("\n=== Test: Single Task ===\n");
    
    TEST_ASSERT(task_scheduler_init(2) == 0);
    
    g_tasks_completed = 0;
    
    /* 创建并提交任务 */
    task_ctx_t* task = task_ctx_create(TASK_TYPE_BG, TASK_PRIO_NORMAL,
                                        test_task_execute, NULL, NULL);
    TEST_ASSERT(task != NULL);
    TEST_ASSERT(task_submit(task, 0) == 0);
    
    /* 等待任务完成 */
    sleep(1);
    
    TEST_ASSERT(g_tasks_completed == 1);
    
    task_scheduler_destroy();
    
    TEST_PASS("Single Task");
    return 0;
}

/* 测试3: 批量任务提交 */
static int test_batch_tasks(void) {
    printf("\n=== Test: Batch Tasks ===\n");
    
    const uint32_t NUM_TASKS = 100;
    
    TEST_ASSERT(task_scheduler_init(4) == 0);
    
    g_tasks_completed = 0;
    
    /* 创建任务数组 */
    task_ctx_t* tasks[NUM_TASKS];
    for (uint32_t i = 0; i < NUM_TASKS; i++) {
        tasks[i] = task_ctx_create(TASK_TYPE_BG, TASK_PRIO_NORMAL,
                                   test_task_execute, NULL, NULL);
        TEST_ASSERT(tasks[i] != NULL);
    }
    
    /* 批量提交 */
    int submitted = task_submit_batch(tasks, NUM_TASKS, -1);
    TEST_ASSERT(submitted == NUM_TASKS);
    
    /* 等待所有任务完成 */
    sleep(2);
    
    TEST_ASSERT(g_tasks_completed == NUM_TASKS);
    
    for (uint32_t i = 0; i < NUM_TASKS; i++) {
        task_ctx_free(tasks[i]);
    }
    
    task_scheduler_destroy();
    
    TEST_PASS("Batch Tasks");
    return 0;
}

/* 测试4: 优先级调度 */
static volatile uint64_t g_high_prio_count = 0;
static volatile uint64_t g_low_prio_count = 0;

static void high_prio_task(task_ctx_t* task) {
    g_high_prio_count++;
}

static void low_prio_task(task_ctx_t* task) {
    g_low_prio_count++;
}

static int test_priority(void) {
    printf("\n=== Test: Priority Scheduling ===\n");
    
    TEST_ASSERT(task_scheduler_init(1) == 0); /* 单核便于测试 */
    
    g_high_prio_count = 0;
    g_low_prio_count = 0;
    
    /* 先提交大量低优先级任务 */
    for (int i = 0; i < 10; i++) {
        task_ctx_t* task = task_ctx_create(TASK_TYPE_BG, TASK_PRIO_LOW,
                                           low_prio_task, NULL, NULL);
        task_submit(task, 0);
    }
    
    usleep(10000); /* 10ms */
    
    /* 提交高优先级任务 */
    for (int i = 0; i < 5; i++) {
        task_ctx_t* task = task_ctx_create(TASK_TYPE_ADMIN, TASK_PRIO_HIGHEST,
                                           high_prio_task, NULL, NULL);
        task_submit(task, 0);
    }
    
    sleep(1);
    
    /* 高优先级任务应该先完成 */
    printf("  High prio completed: %lu\n", g_high_prio_count);
    printf("  Low prio completed: %lu\n", g_low_prio_count);
    TEST_ASSERT(g_high_prio_count == 5);
    
    task_scheduler_destroy();
    
    TEST_PASS("Priority Scheduling");
    return 0;
}

/* 测试5: 任务取消 */
static int test_cancel(void) {
    printf("\n=== Test: Task Cancel ===\n");
    
    TEST_ASSERT(task_scheduler_init(2) == 0);
    
    g_tasks_completed = 0;
    
    /* 创建任务但不执行 */
    task_ctx_t* tasks[10];
    for (int i = 0; i < 10; i++) {
        tasks[i] = task_ctx_create(TASK_TYPE_BG, TASK_PRIO_LOWEST,
                                   test_task_execute, NULL, NULL);
        task_submit(tasks[i], 0);
    }
    
    /* 取消一半任务 */
    for (int i = 0; i < 5; i++) {
        int ret = task_cancel(tasks[i]->task_id);
        TEST_ASSERT(ret == 0);
    }
    
    sleep(1);
    
    /* 应该只有5个任务完成 */
    printf("  Tasks completed: %lu (expected 5)\n", g_tasks_completed);
    TEST_ASSERT(g_tasks_completed == 5);
    
    for (int i = 0; i < 10; i++) {
        task_ctx_free(tasks[i]);
    }
    
    task_scheduler_destroy();
    
    TEST_PASS("Task Cancel");
    return 0;
}

/* 测试6: 队列深度查询 */
static int test_queue_depth(void) {
    printf("\n=== Test: Queue Depth ===\n");
    
    TEST_ASSERT(task_scheduler_init(2) == 0);
    
    /* 提交任务但不启动worker */
    task_ctx_t* tasks[50];
    for (int i = 0; i < 50; i++) {
        tasks[i] = task_ctx_create(TASK_TYPE_BG, TASK_PRIO_LOWEST,
                                   test_task_execute, NULL, NULL);
        task_submit(tasks[i], 0); /* 都提交到core 0 */
    }
    
    usleep(10000);
    
    /* 检查队列深度 */
    uint32_t depth = task_queue_depth(0);
    printf("  Queue depth (core 0): %u\n", depth);
    TEST_ASSERT(depth > 0);
    
    uint32_t total_depth = task_queue_depth(-1);
    printf("  Total queue depth: %u\n", total_depth);
    TEST_ASSERT(total_depth >= depth);
    
    task_scheduler_destroy();
    
    for (int i = 0; i < 50; i++) {
        task_ctx_free(tasks[i]);
    }
    
    TEST_PASS("Queue Depth");
    return 0;
}

/* 测试7: 统计信息 */
static int test_statistics(void) {
    printf("\n=== Test: Statistics ===\n");
    
    TEST_ASSERT(task_scheduler_init(4) == 0);
    
    g_tasks_completed = 0;
    
    /* 提交任务 */
    for (int i = 0; i < 20; i++) {
        task_ctx_t* task = task_ctx_create(TASK_TYPE_BG, TASK_PRIO_NORMAL,
                                           test_task_execute, NULL, NULL);
        task_submit(task, -1);
    }
    
    sleep(2);
    
    /* 获取统计 */
    task_scheduler_stats_t stats;
    TEST_ASSERT(task_get_stats(-1, &stats) == 0);
    
    printf("  Total executed: %lu\n", stats.tasks_executed);
    printf("  Tasks stolen: %lu\n", stats.tasks_stolen);
    printf("  Avg latency: %.2f ns\n", stats.avg_latency_ns);
    
    TEST_ASSERT(stats.tasks_executed == 20);
    TEST_ASSERT(stats.avg_latency_ns > 0);
    
    task_scheduler_destroy();
    
    TEST_PASS("Statistics");
    return 0;
}

/* 测试8: Work Stealing */
static int test_work_stealing(void) {
    printf("\n=== Test: Work Stealing ===\n");
    
    /* 使用4核，但只向core 0提交任务 */
    TEST_ASSERT(task_scheduler_init(4) == 0);
    
    g_tasks_completed = 0;
    
    /* 大量任务只提交到一个核心 */
    for (int i = 0; i < 100; i++) {
        task_ctx_t* task = task_ctx_create(TASK_TYPE_BG, TASK_PRIO_NORMAL,
                                           test_task_execute, NULL, NULL);
        task_submit(task, 0); /* 只提交到core 0 */
    }
    
    sleep(3);
    
    /* 获取统计 */
    task_scheduler_stats_t stats;
    task_get_stats(-1, &stats);
    
    printf("  Total executed: %lu\n", stats.tasks_executed);
    printf("  Tasks stolen: %lu\n", stats.tasks_stolen);
    
    /* 应该有任务被偷取 */
    TEST_ASSERT(stats.tasks_stolen > 0);
    TEST_ASSERT(g_tasks_completed == 100);
    
    task_scheduler_destroy();
    
    TEST_PASS("Work Stealing");
    return 0;
}

/* 主函数 */
int main(int argc, char* argv[]) {
    printf("============================================\n");
    printf("  Task Scheduler Unit Tests\n");
    printf("============================================\n");
    
    srand(time(NULL));
    
    int failures = 0;
    
    if (test_init_destroy() != 0) failures++;
    if (test_single_task() != 0) failures++;
    if (test_batch_tasks() != 0) failures++;
    if (test_priority() != 0) failures++;
    if (test_cancel() != 0) failures++;
    if (test_queue_depth() != 0) failures++;
    if (test_statistics() != 0) failures++;
    if (test_work_stealing() != 0) failures++;
    
    printf("\n============================================\n");
    if (failures == 0) {
        printf("  All tests PASSED!\n");
    } else {
        printf("  %d test(s) FAILED!\n", failures);
    }
    printf("============================================\n");
    
    return failures;
}
