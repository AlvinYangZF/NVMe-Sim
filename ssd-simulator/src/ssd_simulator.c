/**
 * @file ssd_simulator.c
 * @brief SSD Simulator - Main Application
 * 
 * 完整的SSD模拟器主程序，初始化所有模块并提供命令行接口
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include "log_manager.h"
#include "task_scheduler.h"
#include "ddr_manager.h"
#include "nand_simulator.h"
#include "l2p_manager.h"
#include "pcie_controller.h"
#include "nvme_controller.h"
#include "cache_manager.h"
#include "gc_manager.h"
#include "block_device.h"

/* 全局实例 */
typedef struct ssd_context {
    nand_simulator_t* nand;
    l2p_manager_t* l2p;
    pcie_controller_t* pcie;
    nvme_controller_t* nvme;
    cache_manager_t* cache;
    gc_manager_t* gc;
    block_device_t* block_dev;
} ssd_context_t;

static ssd_context_t g_ssd = {0};
static volatile int g_running = 1;

/* 信号处理 */
static void signal_handler(int sig) {
    LOG_WARN("Received signal %d, shutting down...", sig);
    g_running = 0;
}

/* 打印使用说明 */
static void print_usage(const char* prog) {
    printf("Usage: %s [options]\n", prog);
    printf("Options:\n");
    printf("  -c <cores>    Number of CPU cores (default: 4)\n");
    printf("  -s <size>     SSD capacity in GB (default: 1)\n");
    printf("  -t <type>     NAND type: SLC|MLC|TLC|QLC (default: TLC)\n");
    printf("  -h            Show this help\n");
    printf("\nCommands:\n");
    printf("  info          Show SSD information\n");
    printf("  test          Run self-test\n");
    printf("  benchmark     Run performance benchmark\n");
}

/* 初始化SSD */
static int ssd_init(int num_cores, int capacity_gb, nand_type_t nand_type) {
    /* 0. 初始化日志管理器 */
    log_config_t log_cfg = log_get_default_config();
    log_cfg.level = LOG_LEVEL_DEBUG;
    log_cfg.output = LOG_OUTPUT_BOTH;
    log_cfg.enable_timestamp = true;
    log_cfg.enable_location = true;
    log_cfg.max_file_size = 50;  /* 50MB */
    log_cfg.max_files = 5;
    
    if (log_init(&log_cfg) != 0) {
        fprintf(stderr, "Failed to init log manager\n");
        return -1;
    }
    
    LOG_INFO("========================================");
    LOG_INFO("  SSD Simulator - Initialization");
    LOG_INFO("========================================");
    
    /* 1. 初始化DDR管理器 - 使用较小内存 */
    LOG_INFO("Initializing DDR manager...");
    if (ddr_manager_init(64 * 1024 * 1024) != 0) {
        LOG_FATAL("Failed to init DDR manager");
        return -1;
    }
    LOG_INFO("DDR manager initialized: 64 MB");
    
    /* 2. 初始化任务调度器 */
    LOG_INFO("Initializing task scheduler with %d cores...", num_cores);
    if (task_scheduler_init(num_cores) != 0) {
        LOG_FATAL("Failed to init task scheduler");
        goto err_ddr;
    }
    LOG_INFO("Task scheduler initialized with %d cores", num_cores);
    
    /* 3. 创建NAND模拟器 - 使用小配置以减少内存 */
    LOG_INFO("Creating NAND simulator (type=%d, channels=2)...", nand_type);
    nand_config_t nand_cfg = {
        .type = nand_type,
        .num_channels = 2,
        .dies_per_channel = 2,
        .planes_per_die = 2,
        .blocks_per_plane = 16,
        .page_size = 16384,
        .oob_size = 1664,
    };
    g_ssd.nand = nand_sim_create(&nand_cfg);
    if (!g_ssd.nand) {
        LOG_FATAL("Failed to create NAND simulator");
        goto err_scheduler;
    }
    LOG_INFO("NAND simulator created successfully");
    
    /* 4. 创建L2P管理器 - 使用较小配置 */
    LOG_INFO("Creating L2P manager...");
    g_ssd.l2p = l2p_manager_create(1024 * 64, 512);  /* 64K entries, 512 cache */
    if (!g_ssd.l2p) {
        LOG_FATAL("Failed to create L2P manager");
        goto err_nand;
    }
    LOG_INFO("L2P manager created");
    
    /* 5. 创建PCIe控制器 */
    LOG_INFO("Creating PCIe controller...");
    pcie_config_t pcie_cfg = {
        .version = 4,
        .lanes = 4,
        .max_payload = 256,
        .max_read_req = 512,
    };
    g_ssd.pcie = pcie_controller_create(&pcie_cfg);
    if (!g_ssd.pcie) {
        LOG_FATAL("Failed to create PCIe controller");
        goto err_l2p;
    }
    LOG_INFO("PCIe controller created (Gen4 x4)");
    
    /* 6. 创建NVMe控制器 */
    LOG_INFO("Creating NVMe controller...");
    nvme_config_t nvme_cfg = {
        .num_queues = 8,
        .queue_size = 256,
        .max_namespaces = 1,
    };
    g_ssd.nvme = nvme_controller_create(g_ssd.pcie, g_ssd.nand, g_ssd.l2p, &nvme_cfg);
    if (!g_ssd.nvme) {
        LOG_FATAL("Failed to create NVMe controller");
        goto err_pcie;
    }
    LOG_INFO("NVMe controller created (8 queues)");
    
    /* 7. 创建Cache管理器 - 使用较小配置 */
    LOG_INFO("Creating cache manager...");
    g_ssd.cache = cache_manager_create(64, 16384);  /* 64 pages, 16KB each = 1MB */
    if (!g_ssd.cache) {
        LOG_FATAL("Failed to create cache manager");
        goto err_nvme;
    }
    LOG_INFO("Cache manager created (64 pages)");
    
    /* 8. 创建GC管理器 */
    LOG_INFO("Creating GC manager...");
    gc_config_t gc_cfg = {
        .policy = GC_POLICY_GREEDY,
        .threshold = 30,
        .min_free_blocks = 10,
        .max_victim_blocks = 16,
        .enable_background = 1,
    };
    g_ssd.gc = gc_manager_create(g_ssd.nand, g_ssd.l2p, &gc_cfg);
    if (!g_ssd.gc) {
        LOG_FATAL("Failed to create GC manager");
        goto err_cache;
    }
    LOG_INFO("GC manager created (greedy policy)");
    
    /* 9. 创建块设备 */
    LOG_INFO("Creating block device...");
    g_ssd.block_dev = block_device_create(g_ssd.nvme, 1, "ssd0");
    if (!g_ssd.block_dev) {
        LOG_FATAL("Failed to create block device");
        goto err_gc;
    }
    LOG_INFO("Block device 'ssd0' created");
    
    /* 10. 启动NVMe控制器 */
    LOG_INFO("Starting NVMe controller...");
    if (nvme_start(g_ssd.nvme) != 0) {
        LOG_FATAL("Failed to start NVMe controller");
        goto err_block;
    }
    LOG_INFO("NVMe controller started");
    
    LOG_INFO("========================================");
    LOG_INFO("  SSD Initialization Completed!");
    LOG_INFO("========================================");
    LOG_INFO("Cores: %d", num_cores);
    LOG_INFO("NAND Type: %s",
           nand_type == NAND_TYPE_SLC ? "SLC" :
           nand_type == NAND_TYPE_MLC ? "MLC" :
           nand_type == NAND_TYPE_TLC ? "TLC" : "QLC");
    
    return 0;

err_block:
    block_device_destroy(g_ssd.block_dev);
err_gc:
    gc_manager_destroy(g_ssd.gc);
err_cache:
    cache_manager_destroy(g_ssd.cache);
err_nvme:
    nvme_controller_destroy(g_ssd.nvme);
err_pcie:
    pcie_controller_destroy(g_ssd.pcie);
err_l2p:
    l2p_manager_destroy(g_ssd.l2p);
err_nand:
    nand_sim_destroy(g_ssd.nand);
err_scheduler:
    task_scheduler_destroy();
err_ddr:
    ddr_manager_destroy();
    return -1;
}

/* 销毁SSD */
static void ssd_destroy(void) {
    LOG_INFO("========================================");
    LOG_INFO("  SSD Shutting down...");
    LOG_INFO("========================================");
    
    if (g_ssd.nvme) {
        LOG_INFO("Stopping NVMe controller...");
        nvme_stop(g_ssd.nvme);
    }
    
    LOG_DEBUG("Destroying block device...");
    block_device_destroy(g_ssd.block_dev);
    
    LOG_DEBUG("Destroying GC manager...");
    gc_manager_destroy(g_ssd.gc);
    
    LOG_DEBUG("Destroying cache manager...");
    cache_manager_destroy(g_ssd.cache);
    
    LOG_DEBUG("Destroying NVMe controller...");
    nvme_controller_destroy(g_ssd.nvme);
    
    LOG_DEBUG("Destroying PCIe controller...");
    pcie_controller_destroy(g_ssd.pcie);
    
    LOG_DEBUG("Destroying L2P manager...");
    l2p_manager_destroy(g_ssd.l2p);
    
    LOG_DEBUG("Destroying NAND simulator...");
    nand_sim_destroy(g_ssd.nand);
    
    LOG_DEBUG("Destroying task scheduler...");
    task_scheduler_destroy();
    
    LOG_DEBUG("Destroying DDR manager...");
    ddr_manager_destroy();
    
    LOG_INFO("SSD shutdown completed");
    
    /* 最后销毁日志管理器 */
    log_destroy();
}

/* 显示信息 */
static void ssd_info(void) {
    printf("\n========================================\n");
    printf("  SSD Simulator - System Information\n");
    printf("========================================\n");
    
    nand_dump_info(g_ssd.nand);
    l2p_dump_info(g_ssd.l2p);
    pcie_dump_info(g_ssd.pcie);
    nvme_dump_info(g_ssd.nvme);
    cache_dump_info(g_ssd.cache);
    gc_dump_info(g_ssd.gc);
    block_device_dump_info(g_ssd.block_dev);
}

/* 自测试 */
static int ssd_self_test(void) {
    printf("\n========================================\n");
    printf("  SSD Simulator - Self Test\n");
    printf("========================================\n\n");
    
    int errors = 0;
    
    /* 测试1: 简单写读 */
    printf("Test 1: Simple Write/Read...\n");
    
    uint8_t write_buf[4096];
    uint8_t read_buf[4096];
    
    for (int i = 0; i < 4096; i++) {
        write_buf[i] = i & 0xFF;
    }
    
    /* 写入 */
    if (block_device_write(g_ssd.block_dev, 0, 8, write_buf) != 0) {
        printf("  FAILED: Write error\n");
        errors++;
    } else {
        printf("  Write: OK\n");
    }
    
    /* 读取 */
    memset(read_buf, 0, sizeof(read_buf));
    if (block_device_read(g_ssd.block_dev, 0, 8, read_buf) != 0) {
        printf("  FAILED: Read error\n");
        errors++;
    } else {
        printf("  Read: OK\n");
    }
    
    /* 验证 */
    if (memcmp(write_buf, read_buf, 4096) != 0) {
        printf("  FAILED: Data mismatch\n");
        errors++;
    } else {
        printf("  Verify: OK\n");
    }
    
    /* 测试2: 多LBA写读 */
    printf("\nTest 2: Multi-LBA Write/Read...\n");
    
    for (int lba = 0; lba < 100; lba += 10) {
        memset(write_buf, lba, sizeof(write_buf));
        
        if (block_device_write(g_ssd.block_dev, lba, 8, write_buf) != 0) {
            printf("  FAILED: Write LBA %d\n", lba);
            errors++;
            continue;
        }
        
        memset(read_buf, 0, sizeof(read_buf));
        if (block_device_read(g_ssd.block_dev, lba, 8, read_buf) != 0) {
            printf("  FAILED: Read LBA %d\n", lba);
            errors++;
            continue;
        }
        
        if (memcmp(write_buf, read_buf, 4096) != 0) {
            printf("  FAILED: Data mismatch at LBA %d\n", lba);
            errors++;
        }
    }
    
    printf("  Multi-LBA test: %s\n", errors == 0 ? "OK" : "FAILED");
    
    /* 测试3: Flush */
    printf("\nTest 3: Flush...\n");
    if (block_device_flush(g_ssd.block_dev) != 0) {
        printf("  FAILED\n");
        errors++;
    } else {
        printf("  Flush: OK\n");
    }
    
    printf("\n========================================\n");
    if (errors == 0) {
        printf("  All tests PASSED!\n");
    } else {
        printf("  %d test(s) FAILED!\n", errors);
    }
    printf("========================================\n");
    
    return errors;
}

/* 性能测试 */
static void ssd_benchmark(void) {
    printf("\n========================================\n");
    printf("  SSD Simulator - Performance Benchmark\n");
    printf("========================================\n\n");
    
    const int num_ops = 1000;
    const int block_size = 4096;
    
    uint8_t* buf = ddr_alloc(block_size, 4096);
    if (!buf) {
        fprintf(stderr, "Failed to allocate buffer\n");
        return;
    }
    
    struct timespec start, end;
    
    /* 顺序写测试 */
    printf("Sequential Write Test (%d ops, %d KB each)...\n", num_ops, block_size/1024);
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    for (int i = 0; i < num_ops; i++) {
        memset(buf, i, block_size);
        block_device_write(g_ssd.block_dev, i * 8, 8, buf);
    }
    
    clock_gettime(CLOCK_MONOTONIC, &end);
    double write_time = (end.tv_sec - start.tv_sec) + 
                        (end.tv_nsec - start.tv_nsec) / 1e9;
    double write_bw = (double)num_ops * block_size / write_time / (1024 * 1024);
    double write_iops = num_ops / write_time;
    
    printf("  Time: %.3f s\n", write_time);
    printf("  Bandwidth: %.2f MB/s\n", write_bw);
    printf("  IOPS: %.0f\n", write_iops);
    
    /* 顺序读测试 */
    printf("\nSequential Read Test (%d ops, %d KB each)...\n", num_ops, block_size/1024);
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    for (int i = 0; i < num_ops; i++) {
        block_device_read(g_ssd.block_dev, i * 8, 8, buf);
    }
    
    clock_gettime(CLOCK_MONOTONIC, &end);
    double read_time = (end.tv_sec - start.tv_sec) + 
                       (end.tv_nsec - start.tv_nsec) / 1e9;
    double read_bw = (double)num_ops * block_size / read_time / (1024 * 1024);
    double read_iops = num_ops / read_time;
    
    printf("  Time: %.3f s\n", read_time);
    printf("  Bandwidth: %.2f MB/s\n", read_bw);
    printf("  IOPS: %.0f\n", read_iops);
    
    ddr_free(buf);
    
    printf("\n========================================\n");
    printf("  Benchmark completed\n");
    printf("========================================\n");
}

/* 主函数 */
int main(int argc, char* argv[]) {
    int num_cores = 4;
    int capacity_gb = 1;
    nand_type_t nand_type = NAND_TYPE_TLC;
    const char* command = NULL;
    
    /* 解析参数 */
    int opt;
    while ((opt = getopt(argc, argv, "c:s:t:h")) != -1) {
        switch (opt) {
            case 'c':
                num_cores = atoi(optarg);
                break;
            case 's':
                capacity_gb = atoi(optarg);
                break;
            case 't':
                if (strcmp(optarg, "SLC") == 0) nand_type = NAND_TYPE_SLC;
                else if (strcmp(optarg, "MLC") == 0) nand_type = NAND_TYPE_MLC;
                else if (strcmp(optarg, "TLC") == 0) nand_type = NAND_TYPE_TLC;
                else if (strcmp(optarg, "QLC") == 0) nand_type = NAND_TYPE_QLC;
                break;
            case 'h':
            default:
                print_usage(argv[0]);
                return 0;
        }
    }
    
    if (optind < argc) {
        command = argv[optind];
    }
    
    /* 设置信号处理 */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    /* 初始化 */
    if (ssd_init(num_cores, capacity_gb, nand_type) != 0) {
        fprintf(stderr, "SSD initialization failed\n");
        return 1;
    }
    
    /* 执行命令 */
    if (command == NULL || strcmp(command, "info") == 0) {
        ssd_info();
    } else if (strcmp(command, "test") == 0) {
        ssd_self_test();
    } else if (strcmp(command, "benchmark") == 0) {
        ssd_benchmark();
    } else {
        printf("Unknown command: %s\n", command);
        print_usage(argv[0]);
    }
    
    /* 清理 */
    ssd_destroy();
    
    return 0;
}
