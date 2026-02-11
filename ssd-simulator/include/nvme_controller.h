/**
 * @file nvme_controller.h
 * @brief SSD Simulator - NVMe Controller Header
 */

#ifndef __NVME_CONTROLLER_H__
#define __NVME_CONTROLLER_H__

#include <stdint.h>
#include <stdbool.h>
#include "pcie_controller.h"
#include "nand_simulator.h"
#include "l2p_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* NVMe命令 */
typedef struct nvme_command {
    uint8_t  opcode;
    uint8_t  flags;
    uint16_t cid;           /* Command ID */
    uint32_t nsid;          /* Namespace ID */
    uint64_t reserved;
    uint64_t mptr;          /* Metadata Pointer */
    uint64_t prp1;          /* PRP Entry 1 */
    uint64_t prp2;          /* PRP Entry 2 */
    uint32_t cdw10;         /* Command Dword 10 */
    uint32_t cdw11;
    uint32_t cdw12;
    uint32_t cdw13;
    uint32_t cdw14;
    uint32_t cdw15;
} __attribute__((packed)) nvme_command_t;

/* NVMe完成条目 */
typedef struct nvme_completion {
    uint32_t result;
    uint32_t reserved;
    uint16_t sq_head;
    uint16_t sq_id;
    uint16_t cid;
    uint16_t status;
} __attribute__((packed)) nvme_completion_t;

/* NVMe状态码 */
#define NVME_SC_SUCCESS         0x00
#define NVME_SC_INVALID_OPCODE  0x01
#define NVME_SC_INVALID_FIELD   0x02
#define NVME_SC_DATA_XFER_ERROR 0x04
#define NVME_SC_POWER_LOSS      0x05
#define NVME_SC_INTERNAL        0x06
#define NVME_SC_ABORT_REQ       0x07
#define NVME_SC_ABORT_SQ_DEL    0x08
#define NVME_SC_ABORT_FUSE      0x09
#define NVME_SC_ABORT_MISSING   0x0A
#define NVME_SC_INVALID_NS      0x0B
#define NVME_SC_LBA_RANGE       0x80
#define NVME_SC_CAP_EXCEEDED    0x81
#define NVME_SC_NS_NOT_READY    0x82
#define NVME_SC_WRITE_FAULT     0x280
#define NVME_SC_READ_ERROR      0x281

/* 队列 */
typedef struct nvme_queue {
    uint16_t qid;
    uint16_t size;
    uint64_t base_addr;
    uint16_t head;
    uint16_t tail;
    uint16_t cq_id;         /* 关联的CQ（仅SQ） */
    pthread_spinlock_t lock;
} nvme_queue_t;

/* Namespace */
typedef struct nvme_namespace {
    uint32_t nsid;
    uint64_t size;          /* 容量（扇区数） */
    uint32_t lba_size;      /* LBA大小 */
    uint64_t capacity;      /* 最大容量 */
} nvme_namespace_t;

/* NVMe配置 */
typedef struct nvme_config {
    uint32_t num_queues;        /* IO队列对数 */
    uint32_t queue_size;        /* 队列大小 */
    uint32_t max_namespaces;    /* 最大Namespace数 */
} nvme_config_t;

/* NVMe统计 */
typedef struct nvme_stats {
    uint64_t commands_completed;
    uint64_t commands_aborted;
    uint64_t data_read;
    uint64_t data_written;
    uint64_t io_read_ops;
    uint64_t io_write_ops;
} nvme_stats_t;

/* Opaque类型 */
typedef struct nvme_controller nvme_controller_t;

/**
 * @brief 创建NVMe控制器
 * @param pcie PCIe控制器
 * @param nand NAND模拟器
 * @param l2p L2P管理器
 * @param config NVMe配置
 * @return 控制器句柄，NULL失败
 */
nvme_controller_t* nvme_controller_create(pcie_controller_t* pcie,
                                           nand_simulator_t* nand,
                                           l2p_manager_t* l2p,
                                           nvme_config_t* config);

/**
 * @brief 销毁NVMe控制器
 * @param nvme 控制器句柄
 */
void nvme_controller_destroy(nvme_controller_t* nvme);

/**
 * @brief 创建IO队列对
 * @param nvme 控制器句柄
 * @param qid 队列ID
 * @param size 队列大小
 * @return 0成功
 */
int nvme_create_io_queue(nvme_controller_t* nvme, uint16_t qid, uint16_t size);

/**
 * @brief 删除IO队列
 * @param nvme 控制器句柄
 * @param qid 队列ID
 * @return 0成功
 */
int nvme_delete_io_queue(nvme_controller_t* nvme, uint16_t qid);

/**
 * @brief 创建Namespace
 * @param nvme 控制器句柄
 * @param nsid Namespace ID
 * @param size 容量（扇区数）
 * @param lba_size LBA大小
 * @return 0成功
 */
int nvme_create_namespace(nvme_controller_t* nvme, uint32_t nsid, 
                          uint64_t size, uint32_t lba_size);

/**
 * @brief 处理Admin命令
 * @param nvme 控制器句柄
 * @param cmd 命令
 * @param cpl 完成条目输出
 * @return 0成功
 */
int nvme_process_admin_cmd(nvme_controller_t* nvme, 
                           nvme_command_t* cmd, 
                           nvme_completion_t* cpl);

/**
 * @brief 处理IO命令
 * @param nvme 控制器句柄
 * @param qid 队列ID
 * @param cmd 命令
 * @param cpl 完成条目输出
 * @return 0成功
 */
int nvme_process_io_cmd(nvme_controller_t* nvme, uint16_t qid,
                        nvme_command_t* cmd, 
                        nvme_completion_t* cpl);

/**
 * @brief 启动控制器
 * @param nvme 控制器句柄
 * @return 0成功
 */
int nvme_start(nvme_controller_t* nvme);

/**
 * @brief 停止控制器
 * @param nvme 控制器句柄
 * @return 0成功
 */
int nvme_stop(nvme_controller_t* nvme);

/**
 * @brief 获取统计信息
 * @param nvme 控制器句柄
 * @param stats 统计输出
 */
void nvme_get_stats(nvme_controller_t* nvme, nvme_stats_t* stats);

/**
 * @brief 打印NVMe信息
 * @param nvme 控制器句柄
 */
void nvme_dump_info(nvme_controller_t* nvme);

#ifdef __cplusplus
}
#endif

#endif /* __NVME_CONTROLLER_H__ */
