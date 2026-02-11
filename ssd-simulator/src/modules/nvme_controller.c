/**
 * @file nvme_controller.c
 * @brief SSD Simulator - NVMe Controller Implementation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include "nvme_controller.h"
#include "ddr_manager.h"

/* NVMe操作码 */
#define NVME_CMD_FLUSH          0x00
#define NVME_CMD_WRITE          0x01
#define NVME_CMD_READ           0x02
#define NVME_CMD_WRITE_UNCORR   0x04
#define NVME_CMD_COMPARE        0x05
#define NVME_CMD_WRITE_ZEROES   0x08
#define NVME_CMD_DSM            0x09

/* Admin命令 */
#define NVME_ADM_DELETE_SQ      0x00
#define NVME_ADM_CREATE_SQ      0x01
#define NVME_ADM_GET_LOG_PAGE   0x02
#define NVME_ADM_DELETE_CQ      0x04
#define NVME_ADM_CREATE_CQ      0x05
#define NVME_ADM_IDENTIFY       0x06
#define NVME_ADM_ABORT          0x08
#define NVME_ADM_SET_FEATURES   0x09
#define NVME_ADM_GET_FEATURES   0x0A

/* NVMe控制器 */
struct nvme_controller {
    /* 下层接口 */
    pcie_controller_t* pcie;
    nand_simulator_t* nand;
    l2p_manager_t* l2p;
    
    nvme_config_t config;
    
    /* 队列 */
    nvme_queue_t* admin_sq;
    nvme_queue_t* admin_cq;
    nvme_queue_t** io_sqs;
    nvme_queue_t** io_cqs;
    
    /* Namespace */
    nvme_namespace_t** namespaces;
    uint32_t num_namespaces;
    
    /* 状态 */
    uint8_t ready;
    volatile int running;
    pthread_t admin_thread;
    pthread_t* io_threads;
    
    /* 统计 */
    nvme_stats_t stats;
    pthread_mutex_t stats_lock;
};

/* 前向声明 */
static void* admin_thread_fn(void* arg);
static void* io_thread_fn(void* arg);
static int handle_admin_create_sq(nvme_controller_t* nvme, nvme_command_t* cmd);
static int handle_admin_create_cq(nvme_controller_t* nvme, nvme_command_t* cmd);
static int handle_io_read(nvme_controller_t* nvme, uint16_t qid, 
                          nvme_command_t* cmd, nvme_completion_t* cpl);
static int handle_io_write(nvme_controller_t* nvme, uint16_t qid,
                           nvme_command_t* cmd, nvme_completion_t* cpl);
static int dma_xfer_prp(nvme_controller_t* nvme, uint64_t prp1, uint64_t prp2,
                        void* buf, size_t len, dma_dir_t dir);

nvme_controller_t* nvme_controller_create(pcie_controller_t* pcie,
                                           nand_simulator_t* nand,
                                           l2p_manager_t* l2p,
                                           nvme_config_t* config) {
    if (!pcie || !nand || !l2p || !config) return NULL;
    
    nvme_controller_t* nvme = calloc(1, sizeof(nvme_controller_t));
    if (!nvme) return NULL;
    
    nvme->pcie = pcie;
    nvme->nand = nand;
    nvme->l2p = l2p;
    memcpy(&nvme->config, config, sizeof(nvme_config_t));
    
    /* 分配队列数组 */
    nvme->io_sqs = calloc(config->num_queues, sizeof(nvme_queue_t*));
    nvme->io_cqs = calloc(config->num_queues, sizeof(nvme_queue_t*));
    nvme->io_threads = calloc(config->num_queues, sizeof(pthread_t));
    
    /* 分配Namespace数组 */
    nvme->namespaces = calloc(config->max_namespaces, sizeof(nvme_namespace_t*));
    
    pthread_mutex_init(&nvme->stats_lock, NULL);
    
    printf("[NVMe] Controller created: %u queues, %u max NS\n",
           config->num_queues, config->max_namespaces);
    
    return nvme;
}

void nvme_controller_destroy(nvme_controller_t* nvme) {
    if (!nvme) return;
    
    nvme_stop(nvme);
    
    /* 清理队列 */
    for (uint32_t i = 0; i < nvme->config.num_queues; i++) {
        if (nvme->io_sqs[i]) {
            pthread_spin_destroy(&nvme->io_sqs[i]->lock);
            free(nvme->io_sqs[i]);
        }
        if (nvme->io_cqs[i]) {
            pthread_spin_destroy(&nvme->io_cqs[i]->lock);
            free(nvme->io_cqs[i]);
        }
    }
    
    /* 清理Namespace */
    for (uint32_t i = 0; i < nvme->config.max_namespaces; i++) {
        free(nvme->namespaces[i]);
    }
    
    pthread_mutex_destroy(&nvme->stats_lock);
    free(nvme->io_sqs);
    free(nvme->io_cqs);
    free(nvme->io_threads);
    free(nvme->namespaces);
    free(nvme);
    
    printf("[NVMe] Controller destroyed\n");
}

int nvme_create_io_queue(nvme_controller_t* nvme, uint16_t qid, uint16_t size) {
    if (!nvme || qid == 0 || qid >= nvme->config.num_queues) return -EINVAL;
    
    /* 创建SQ */
    nvme->io_sqs[qid] = calloc(1, sizeof(nvme_queue_t));
    nvme->io_sqs[qid]->qid = qid;
    nvme->io_sqs[qid]->size = size;
    pthread_spin_init(&nvme->io_sqs[qid]->lock, PTHREAD_PROCESS_PRIVATE);
    
    /* 创建CQ */
    nvme->io_cqs[qid] = calloc(1, sizeof(nvme_queue_t));
    nvme->io_cqs[qid]->qid = qid;
    nvme->io_cqs[qid]->size = size;
    pthread_spin_init(&nvme->io_cqs[qid]->lock, PTHREAD_PROCESS_PRIVATE);
    
    printf("[NVMe] IO queue pair %u created, size %u\n", qid, size);
    return 0;
}

int nvme_delete_io_queue(nvme_controller_t* nvme, uint16_t qid) {
    if (!nvme || qid == 0 || qid >= nvme->config.num_queues) return -EINVAL;
    
    if (nvme->io_sqs[qid]) {
        pthread_spin_destroy(&nvme->io_sqs[qid]->lock);
        free(nvme->io_sqs[qid]);
        nvme->io_sqs[qid] = NULL;
    }
    
    if (nvme->io_cqs[qid]) {
        pthread_spin_destroy(&nvme->io_cqs[qid]->lock);
        free(nvme->io_cqs[qid]);
        nvme->io_cqs[qid] = NULL;
    }
    
    return 0;
}

int nvme_create_namespace(nvme_controller_t* nvme, uint32_t nsid, 
                          uint64_t size, uint32_t lba_size) {
    if (!nvme || nsid == 0 || nsid > nvme->config.max_namespaces) return -EINVAL;
    
    nvme_namespace_t* ns = calloc(1, sizeof(nvme_namespace_t));
    ns->nsid = nsid;
    ns->size = size;
    ns->lba_size = lba_size;
    ns->capacity = size;
    
    nvme->namespaces[nsid - 1] = ns;
    nvme->num_namespaces++;
    
    printf("[NVMe] Namespace %u created: %lu sectors, %u bytes/sector\n",
           nsid, size, lba_size);
    return 0;
}

int nvme_start(nvme_controller_t* nvme) {
    if (!nvme || nvme->running) return -EINVAL;
    
    nvme->running = 1;
    
    /* 启动Admin线程 */
    pthread_create(&nvme->admin_thread, NULL, admin_thread_fn, nvme);
    
    /* 启动IO线程 */
    for (uint32_t i = 1; i < nvme->config.num_queues; i++) {
        if (nvme->io_sqs[i]) {
            pthread_create(&nvme->io_threads[i], NULL, io_thread_fn, nvme);
        }
    }
    
    nvme->ready = 1;
    printf("[NVMe] Controller started\n");
    return 0;
}

int nvme_stop(nvme_controller_t* nvme) {
    if (!nvme || !nvme->running) return -EINVAL;
    
    nvme->running = 0;
    
    /* 等待线程结束 */
    pthread_join(nvme->admin_thread, NULL);
    
    for (uint32_t i = 1; i < nvme->config.num_queues; i++) {
        if (nvme->io_threads[i]) {
            pthread_join(nvme->io_threads[i], NULL);
        }
    }
    
    nvme->ready = 0;
    printf("[NVMe] Controller stopped\n");
    return 0;
}

void nvme_get_stats(nvme_controller_t* nvme, nvme_stats_t* stats) {
    if (!nvme || !stats) return;
    
    pthread_mutex_lock(&nvme->stats_lock);
    memcpy(stats, &nvme->stats, sizeof(nvme_stats_t));
    pthread_mutex_unlock(&nvme->stats_lock);
}

void nvme_dump_info(nvme_controller_t* nvme) {
    if (!nvme) return;
    
    printf("\n[NVMe] Controller Info:\n");
    printf("  State: %s\n", nvme->ready ? "Ready" : "Not Ready");
    printf("  Queues: %u\n", nvme->config.num_queues);
    printf("  Namespaces: %u\n", nvme->num_namespaces);
    
    nvme_stats_t stats;
    nvme_get_stats(nvme, &stats);
    
    printf("\n  Statistics:\n");
    printf("    Commands completed: %lu\n", stats.commands_completed);
    printf("    Read operations: %lu\n", stats.io_read_ops);
    printf("    Write operations: %lu\n", stats.io_write_ops);
    printf("    Data read: %lu MB\n", stats.data_read / (1024*1024));
    printf("    Data written: %lu MB\n", stats.data_written / (1024*1024));
}

/* Admin线程 */
static void* admin_thread_fn(void* arg) {
    nvme_controller_t* nvme = (nvme_controller_t*)arg;
    
    printf("[NVMe] Admin thread started\n");
    
    while (nvme->running) {
        /* 实际实现会从Admin SQ获取命令并处理 */
        /* 这里简化处理 */
        usleep(1000);
    }
    
    printf("[NVMe] Admin thread stopped\n");
    return NULL;
}

/* IO线程 */
static void* io_thread_fn(void* arg) {
    nvme_controller_t* nvme = (nvme_controller_t*)arg;
    
    printf("[NVMe] IO thread started\n");
    
    while (nvme->running) {
        /* 实际实现会从各IO SQ获取命令并处理 */
        usleep(100);
    }
    
    printf("[NVMe] IO thread stopped\n");
    return NULL;
}

/* 处理IO读 */
static int handle_io_read(nvme_controller_t* nvme, uint16_t qid,
                          nvme_command_t* cmd, nvme_completion_t* cpl) {
    uint64_t slba = ((uint64_t)cmd->cdw11 << 32) | cmd->cdw10;
    uint32_t nlb = (cmd->cdw12 & 0xFFFF) + 1;
    uint32_t nsid = cmd->nsid;
    
    if (nsid == 0 || nsid > nvme->config.max_namespaces || 
        !nvme->namespaces[nsid - 1]) {
        cpl->status = (NVME_SC_INVALID_NS << 1) | 1;
        return -1;
    }
    
    nvme_namespace_t* ns = nvme->namespaces[nsid - 1];
    
    /* 验证范围 */
    if (slba + nlb > ns->size) {
        cpl->status = (NVME_SC_LBA_RANGE << 1) | 1;
        return -1;
    }
    
    /* 分配缓冲区 */
    size_t buf_size = nlb * ns->lba_size;
    void* buf = ddr_alloc(buf_size, ns->lba_size);
    if (!buf) {
        cpl->status = (NVME_SC_INTERNAL << 1) | 1;
        return -1;
    }
    
    /* 读取每个LBA */
    for (uint32_t i = 0; i < nlb; i++) {
        uint64_t lba = slba + i;
        
        /* L2P查找 */
        l2p_entry_t entry;
        int ret = l2p_lookup(nvme->l2p, lba, &entry);
        
        if (ret != 0 || !(entry.flags & L2P_FLAG_VALID)) {
            /* 未映射，返回0 */
            memset((char*)buf + i * ns->lba_size, 0, ns->lba_size);
            continue;
        }
        
        /* 从NAND读取 */
        uint64_t ppa = entry.ppa;
        void* page_buf = ddr_alloc(16384, 4096);
        
        nand_read_page(nvme->nand, ppa, page_buf, NULL);
        
        /* 提取LBA数据 */
        memcpy((char*)buf + i * ns->lba_size, page_buf, ns->lba_size);
        
        ddr_free(page_buf);
    }
    
    /* DMA传输到主机 */
    dma_xfer_prp(nvme, cmd->prp1, cmd->prp2, buf, buf_size, DMA_DIR_FROM_DEVICE);
    
    ddr_free(buf);
    
    pthread_mutex_lock(&nvme->stats_lock);
    nvme->stats.io_read_ops++;
    nvme->stats.data_read += buf_size;
    nvme->stats.commands_completed++;
    pthread_mutex_unlock(&nvme->stats_lock);
    
    cpl->status = 0;
    return 0;
}

/* 处理IO写 */
static int handle_io_write(nvme_controller_t* nvme, uint16_t qid,
                           nvme_command_t* cmd, nvme_completion_t* cpl) {
    uint64_t slba = ((uint64_t)cmd->cdw11 << 32) | cmd->cdw10;
    uint32_t nlb = (cmd->cdw12 & 0xFFFF) + 1;
    uint32_t nsid = cmd->nsid;
    
    if (nsid == 0 || nsid > nvme->config.max_namespaces || 
        !nvme->namespaces[nsid - 1]) {
        cpl->status = (NVME_SC_INVALID_NS << 1) | 1;
        return -1;
    }
    
    nvme_namespace_t* ns = nvme->namespaces[nsid - 1];
    
    if (slba + nlb > ns->size) {
        cpl->status = (NVME_SC_LBA_RANGE << 1) | 1;
        return -1;
    }
    
    /* 分配缓冲区 */
    size_t buf_size = nlb * ns->lba_size;
    void* buf = ddr_alloc(buf_size, ns->lba_size);
    if (!buf) {
        cpl->status = (NVME_SC_INTERNAL << 1) | 1;
        return -1;
    }
    
    /* DMA从主机读取数据 */
    dma_xfer_prp(nvme, cmd->prp1, cmd->prp2, buf, buf_size, DMA_DIR_TO_DEVICE);
    
    /* 写入每个LBA */
    for (uint32_t i = 0; i < nlb; i++) {
        uint64_t lba = slba + i;
        
        /* 分配PPA */
        uint64_t ppa;
        int ret = l2p_alloc_ppa(nvme->l2p, &ppa);
        if (ret != 0) {
            ddr_free(buf);
            cpl->status = (NVME_SC_CAP_EXCEEDED << 1) | 1;
            return -1;
        }
        
        /* 准备页数据 */
        void* page_buf = ddr_zalloc(16384);
        memcpy(page_buf, (char*)buf + i * ns->lba_size, ns->lba_size);
        
        /* 写入NAND */
        nand_write_page(nvme->nand, ppa, page_buf, NULL);
        
        ddr_free(page_buf);
        
        /* 更新L2P */
        l2p_update(nvme->l2p, lba, ppa, L2P_FLAG_VALID);
    }
    
    ddr_free(buf);
    
    pthread_mutex_lock(&nvme->stats_lock);
    nvme->stats.io_write_ops++;
    nvme->stats.data_written += buf_size;
    nvme->stats.commands_completed++;
    pthread_mutex_unlock(&nvme->stats_lock);
    
    cpl->status = 0;
    return 0;
}

/* DMA传输 */
static int dma_xfer_prp(nvme_controller_t* nvme, uint64_t prp1, uint64_t prp2,
                        void* buf, size_t len, dma_dir_t dir) {
    /* 简化实现：直接调用PCIe DMA */
    dma_descriptor_t desc = {
        .src_addr = (dir == DMA_DIR_TO_DEVICE) ? prp1 : (uint64_t)buf,
        .dst_addr = (dir == DMA_DIR_TO_DEVICE) ? (uint64_t)buf : prp1,
        .length = len,
        .flags = DMA_FLAG_EOL,
        .next_desc = 0
    };
    
    pcie_dma_submit(nvme->pcie, (uint64_t)&desc, 1);
    
    return 0;
}

int nvme_process_admin_cmd(nvme_controller_t* nvme, 
                           nvme_command_t* cmd, 
                           nvme_completion_t* cpl) {
    if (!nvme || !cmd || !cpl) return -EINVAL;
    
    memset(cpl, 0, sizeof(*cpl));
    cpl->cid = cmd->cid;
    cpl->sq_id = 0;
    
    switch (cmd->opcode) {
        case NVME_ADM_CREATE_SQ:
            return handle_admin_create_sq(nvme, cmd);
        case NVME_ADM_CREATE_CQ:
            return handle_admin_create_cq(nvme, cmd);
        default:
            cpl->status = (NVME_SC_INVALID_OPCODE << 1) | 1;
            return -1;
    }
}

int nvme_process_io_cmd(nvme_controller_t* nvme, uint16_t qid,
                        nvme_command_t* cmd, 
                        nvme_completion_t* cpl) {
    if (!nvme || !cmd || !cpl) return -EINVAL;
    
    memset(cpl, 0, sizeof(*cpl));
    cpl->cid = cmd->cid;
    cpl->sq_id = qid;
    
    switch (cmd->opcode) {
        case NVME_CMD_READ:
            return handle_io_read(nvme, qid, cmd, cpl);
        case NVME_CMD_WRITE:
            return handle_io_write(nvme, qid, cmd, cpl);
        case NVME_CMD_FLUSH:
            /* Flush操作 */
            cpl->status = 0;
            return 0;
        default:
            cpl->status = (NVME_SC_INVALID_OPCODE << 1) | 1;
            return -1;
    }
}

static int handle_admin_create_sq(nvme_controller_t* nvme, nvme_command_t* cmd) {
    uint16_t qid = cmd->cdw10 & 0xFFFF;
    uint16_t size = ((cmd->cdw10 >> 16) & 0xFFFF) + 1;
    
    return nvme_create_io_queue(nvme, qid, size);
}

static int handle_admin_create_cq(nvme_controller_t* nvme, nvme_command_t* cmd) {
    /* CQ创建已在nvme_create_io_queue中处理 */
    return 0;
}
