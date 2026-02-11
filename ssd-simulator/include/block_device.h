/**
 * @file block_device.h
 * @brief SSD Simulator - Block Device Interface Header
 */

#ifndef __BLOCK_DEVICE_H__
#define __BLOCK_DEVICE_H__

#include <stdint.h>
#include <stdbool.h>
#include "nvme_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 块设备统计 */
typedef struct block_dev_stats {
    uint64_t read_ops;
    uint64_t write_ops;
    uint64_t read_bytes;
    uint64_t write_bytes;
    uint64_t io_errors;
    double avg_read_latency_us;
    double avg_write_latency_us;
} block_dev_stats_t;

/* Opaque类型 */
typedef struct block_device block_device_t;

/**
 * @brief 创建块设备
 * @param nvme NVMe控制器
 * @param nsid Namespace ID
 * @param device_name 设备名称
 * @return 块设备句柄，NULL失败
 */
block_device_t* block_device_create(nvme_controller_t* nvme, 
                                     uint32_t nsid,
                                     const char* device_name);

/**
 * @brief 销毁块设备
 * @param dev 块设备句柄
 */
void block_device_destroy(block_device_t* dev);

/**
 * @brief 读取数据
 * @param dev 块设备句柄
 * @param lba 起始逻辑块地址
 * @param count 块数
 * @param buf 数据缓冲区
 * @return 0成功，负数错误码
 */
int block_device_read(block_device_t* dev, uint64_t lba, 
                       uint32_t count, void* buf);

/**
 * @brief 写入数据
 * @param dev 块设备句柄
 * @param lba 起始逻辑块地址
 * @param count 块数
 * @param buf 数据缓冲区
 * @return 0成功，负数错误码
 */
int block_device_write(block_device_t* dev, uint64_t lba,
                        uint32_t count, void* buf);

/**
 * @brief 刷新缓存
 * @param dev 块设备句柄
 * @return 0成功
 */
int block_device_flush(block_device_t* dev);

/**
 * @brief 获取设备容量
 * @param dev 块设备句柄
 * @return 容量（字节）
 */
uint64_t block_device_get_capacity(block_device_t* dev);

/**
 * @brief 获取块大小
 * @param dev 块设备句柄
 * @return 块大小（字节）
 */
uint32_t block_device_get_block_size(block_device_t* dev);

/**
 * @brief 获取统计信息
 * @param dev 块设备句柄
 * @param stats 统计输出
 */
void block_device_get_stats(block_device_t* dev, block_dev_stats_t* stats);

/**
 * @brief 打印设备信息
 * @param dev 块设备句柄
 */
void block_device_dump_info(block_device_t* dev);

#ifdef __cplusplus
}
#endif

#endif /* __BLOCK_DEVICE_H__ */
