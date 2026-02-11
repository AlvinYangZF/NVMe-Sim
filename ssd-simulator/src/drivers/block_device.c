/**
 * @file block_device.c
 * @brief SSD Simulator - Block Device Interface Implementation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include "block_device.h"
#include "ddr_manager.h"

/* 块设备 */
struct block_device {
    nvme_controller_t* nvme;
    uint32_t nsid;
    char name[32];
    
    uint64_t capacity;
    uint32_t block_size;
    uint64_t num_blocks;
    
    /* 统计 */
    block_dev_stats_t stats;
    uint64_t total_read_latency;
    uint64_t total_write_latency;
    pthread_mutex_t stats_lock;
};

/* 前向声明 */
static inline uint64_t get_time_us(void);

block_device_t* block_device_create(nvme_controller_t* nvme, 
                                     uint32_t nsid,
                                     const char* device_name) {
    if (!nvme || !device_name) return NULL;
    
    block_device_t* dev = calloc(1, sizeof(block_device_t));
    if (!dev) return NULL;
    
    dev->nvme = nvme;
    dev->nsid = nsid;
    strncpy(dev->name, device_name, sizeof(dev->name) - 1);
    
    /* 从NVMe获取Namespace信息 */
    /* 简化实现：使用固定值 */
    dev->block_size = 512;  /* 标准扇区大小 */
    dev->num_blocks = 1024 * 1024 * 1024 / 512;  /* 1GB */
    dev->capacity = dev->num_blocks * dev->block_size;
    
    pthread_mutex_init(&dev->stats_lock, NULL);
    
    printf("[BlockDev] Device '%s' created: %lu MB, block size %u\n",
           dev->name, dev->capacity / (1024 * 1024), dev->block_size);
    
    return dev;
}

void block_device_destroy(block_device_t* dev) {
    if (!dev) return;
    
    block_device_flush(dev);
    
    pthread_mutex_destroy(&dev->stats_lock);
    free(dev);
    
    printf("[BlockDev] Device destroyed\n");
}

int block_device_read(block_device_t* dev, uint64_t lba, 
                       uint32_t count, void* buf) {
    if (!dev || !buf) return -EINVAL;
    if (lba + count > dev->num_blocks) return -EINVAL;
    
    uint64_t start_time = get_time_us();
    
    /* 构建NVMe读命令 */
    nvme_command_t cmd = {0};
    cmd.opcode = 0x02;  /* Read */
    cmd.nsid = dev->nsid;
    cmd.cdw10 = lba & 0xFFFFFFFF;
    cmd.cdw11 = (lba >> 32) & 0xFFFFFFFF;
    cmd.cdw12 = (count - 1) & 0xFFFF;  /* 0-based count */
    
    /* 分配PRP */
    void* prp_buf = ddr_alloc(count * dev->block_size, 4096);
    if (!prp_buf) return -ENOMEM;
    
    cmd.prp1 = (uint64_t)prp_buf;
    
    /* 提交命令 */
    nvme_completion_t cpl = {0};
    int ret = nvme_process_io_cmd(dev->nvme, 1, &cmd, &cpl);
    
    if (ret == 0 && (cpl.status & 0x1) == 0) {
        /* 复制数据到用户缓冲区 */
        memcpy(buf, prp_buf, count * dev->block_size);
        
        uint64_t latency = get_time_us() - start_time;
        
        pthread_mutex_lock(&dev->stats_lock);
        dev->stats.read_ops++;
        dev->stats.read_bytes += count * dev->block_size;
        dev->total_read_latency += latency;
        dev->stats.avg_read_latency_us = 
            (double)dev->total_read_latency / dev->stats.read_ops;
        pthread_mutex_unlock(&dev->stats_lock);
    } else {
        pthread_mutex_lock(&dev->stats_lock);
        dev->stats.io_errors++;
        pthread_mutex_unlock(&dev->stats_lock);
        ret = -EIO;
    }
    
    ddr_free(prp_buf);
    
    return ret;
}

int block_device_write(block_device_t* dev, uint64_t lba,
                        uint32_t count, void* buf) {
    if (!dev || !buf) return -EINVAL;
    if (lba + count > dev->num_blocks) return -EINVAL;
    
    uint64_t start_time = get_time_us();
    
    /* 分配PRP缓冲区 */
    void* prp_buf = ddr_alloc(count * dev->block_size, 4096);
    if (!prp_buf) return -ENOMEM;
    
    /* 复制用户数据 */
    memcpy(prp_buf, buf, count * dev->block_size);
    
    /* 构建NVMe写命令 */
    nvme_command_t cmd = {0};
    cmd.opcode = 0x01;  /* Write */
    cmd.nsid = dev->nsid;
    cmd.cdw10 = lba & 0xFFFFFFFF;
    cmd.cdw11 = (lba >> 32) & 0xFFFFFFFF;
    cmd.cdw12 = (count - 1) & 0xFFFF;
    cmd.prp1 = (uint64_t)prp_buf;
    
    /* 提交命令 */
    nvme_completion_t cpl = {0};
    int ret = nvme_process_io_cmd(dev->nvme, 1, &cmd, &cpl);
    
    if (ret == 0 && (cpl.status & 0x1) == 0) {
        uint64_t latency = get_time_us() - start_time;
        
        pthread_mutex_lock(&dev->stats_lock);
        dev->stats.write_ops++;
        dev->stats.write_bytes += count * dev->block_size;
        dev->total_write_latency += latency;
        dev->stats.avg_write_latency_us = 
            (double)dev->total_write_latency / dev->stats.write_ops;
        pthread_mutex_unlock(&dev->stats_lock);
    } else {
        pthread_mutex_lock(&dev->stats_lock);
        dev->stats.io_errors++;
        pthread_mutex_unlock(&dev->stats_lock);
        ret = -EIO;
    }
    
    ddr_free(prp_buf);
    
    return ret;
}

int block_device_flush(block_device_t* dev) {
    if (!dev) return -EINVAL;
    
    /* 构建Flush命令 */
    nvme_command_t cmd = {0};
    cmd.opcode = 0x00;  /* Flush */
    cmd.nsid = dev->nsid;
    
    nvme_completion_t cpl = {0};
    return nvme_process_io_cmd(dev->nvme, 1, &cmd, &cpl);
}

uint64_t block_device_get_capacity(block_device_t* dev) {
    return dev ? dev->capacity : 0;
}

uint32_t block_device_get_block_size(block_device_t* dev) {
    return dev ? dev->block_size : 0;
}

void block_device_get_stats(block_device_t* dev, block_dev_stats_t* stats) {
    if (!dev || !stats) return;
    
    pthread_mutex_lock(&dev->stats_lock);
    memcpy(stats, &dev->stats, sizeof(block_dev_stats_t));
    pthread_mutex_unlock(&dev->stats_lock);
}

void block_device_dump_info(block_device_t* dev) {
    if (!dev) return;
    
    block_dev_stats_t stats;
    block_device_get_stats(dev, &stats);
    
    printf("\n[BlockDev] Device '%s' Info:\n", dev->name);
    printf("  Capacity: %lu MB (%lu blocks)\n", 
           dev->capacity / (1024 * 1024), dev->num_blocks);
    printf("  Block size: %u bytes\n", dev->block_size);
    
    printf("\n  Statistics:\n");
    printf("    Read operations:  %lu (%.2f MB)\n", 
           stats.read_ops, stats.read_bytes / (1024.0 * 1024));
    printf("    Write operations: %lu (%.2f MB)\n", 
           stats.write_ops, stats.write_bytes / (1024.0 * 1024));
    printf("    Read latency:  %.2f us avg\n", stats.avg_read_latency_us);
    printf("    Write latency: %.2f us avg\n", stats.avg_write_latency_us);
    printf("    IO errors: %lu\n", stats.io_errors);
}

static inline uint64_t get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;
}
