/**
 * @file pcie_controller.c
 * @brief SSD Simulator - PCIe Controller Implementation
 */

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include "pcie_controller.h"

/* PCIe配置空间 */
typedef struct pcie_config_space {
    /* 标准头 */
    uint16_t vendor_id;
    uint16_t device_id;
    uint16_t command;
    uint16_t status;
    uint8_t  revision;
    uint8_t  prog_if;
    uint8_t  subclass;
    uint8_t  class_code;
    uint8_t  cache_line_size;
    uint8_t  latency_timer;
    uint8_t  header_type;
    uint8_t  bist;
    
    /* BARs */
    uint32_t bar[6];
    
    /* 子系统 */
    uint16_t subsys_vendor;
    uint16_t subsys_id;
    
    /* 中断 */
    uint8_t  int_line;
    uint8_t  int_pin;
} __attribute__((packed)) pcie_config_space_t;

/* DMA引擎 */
typedef struct dma_engine {
    uint8_t  id;
    uint8_t  busy;
    uint64_t desc_addr;
    uint32_t desc_count;
    uint64_t bytes_transferred;
    
    pthread_t thread;
    volatile int running;
} dma_engine_t;

/* DMA线程参数 */
typedef struct dma_thread_arg {
    struct pcie_controller* pcie;
    dma_engine_t* dma;
} dma_thread_arg_t;

/* PCIe控制器 */
struct pcie_controller {
    pcie_config_t config;
    pcie_config_space_t config_space;
    
    /* BAR0基地址 */
    uint64_t bar0_base;
    
    /* DMA引擎 */
    dma_engine_t* dma_engines;
    uint32_t num_dma_engines;
    
    /* 主机回调 */
    host_dma_read_fn_t host_dma_read;
    host_dma_write_fn_t host_dma_write;
    host_interrupt_fn_t host_interrupt;
    
    /* 统计 */
    pcie_stats_t stats;
    pthread_mutex_t stats_lock;
};

/* 速度表 */
static const uint32_t g_pcie_speeds[6] = {
    2500,   /* 1.0: 2.5 GT/s */
    5000,   /* 2.0: 5 GT/s */
    8000,   /* 3.0: 8 GT/s */
    16000,  /* 4.0: 16 GT/s */
    32000,  /* 5.0: 32 GT/s */
    64000,  /* 6.0: 64 GT/s */
};

/* 前向声明 */
static void* dma_thread(void* arg);
static int execute_dma_transfer(pcie_controller_t* pcie, dma_descriptor_t* desc);

pcie_controller_t* pcie_controller_create(pcie_config_t* config) {
    if (!config) return NULL;
    
    pcie_controller_t* pcie = calloc(1, sizeof(pcie_controller_t));
    if (!pcie) return NULL;
    
    memcpy(&pcie->config, config, sizeof(pcie_config_t));
    
    /* 初始化配置空间 */
    pcie->config_space.vendor_id = 0x1234;  /* 模拟Vendor ID */
    pcie->config_space.device_id = 0x5678;  /* 模拟Device ID */
    pcie->config_space.revision = 0x01;
    pcie->config_space.class_code = 0x01;   /* Mass Storage */
    pcie->config_space.subclass = 0x08;     /* NVMe */
    pcie->config_space.prog_if = 0x02;
    pcie->config_space.header_type = 0x00;
    pcie->config_space.int_pin = 0x01;
    
    /* 初始化BARs */
    pcie->config_space.bar[0] = 0x0;  /* 64-bit MMIO，稍后设置 */
    
    /* 初始化DMA引擎 */
    pcie->num_dma_engines = 4;
    pcie->dma_engines = calloc(pcie->num_dma_engines, sizeof(dma_engine_t));
    if (!pcie->dma_engines) {
        free(pcie);
        return NULL;
    }
    
    for (uint32_t i = 0; i < pcie->num_dma_engines; i++) {
        pcie->dma_engines[i].id = i;
        pcie->dma_engines[i].running = 1;
        dma_thread_arg_t* arg = malloc(sizeof(dma_thread_arg_t));
        arg->pcie = pcie;
        arg->dma = &pcie->dma_engines[i];
        pthread_create(&pcie->dma_engines[i].thread, NULL, dma_thread, arg);
    }
    
    pthread_mutex_init(&pcie->stats_lock, NULL);
    
    printf("[PCIe] Controller created: Gen%u x%u\n", 
           pcie->config.version, pcie->config.lanes);
    
    return pcie;
}

void pcie_controller_destroy(pcie_controller_t* pcie) {
    if (!pcie) return;
    
    /* 停止DMA引擎 */
    for (uint32_t i = 0; i < pcie->num_dma_engines; i++) {
        pcie->dma_engines[i].running = 0;
    }
    
    for (uint32_t i = 0; i < pcie->num_dma_engines; i++) {
        pthread_join(pcie->dma_engines[i].thread, NULL);
    }
    
    pthread_mutex_destroy(&pcie->stats_lock);
    free(pcie->dma_engines);
    free(pcie);
    
    printf("[PCIe] Controller destroyed\n");
}

int pcie_register_host_callbacks(pcie_controller_t* pcie,
                                  host_dma_read_fn_t dma_read,
                                  host_dma_write_fn_t dma_write,
                                  host_interrupt_fn_t interrupt) {
    if (!pcie) return -EINVAL;
    
    pcie->host_dma_read = dma_read;
    pcie->host_dma_write = dma_write;
    pcie->host_interrupt = interrupt;
    
    return 0;
}

int pcie_dma_submit(pcie_controller_t* pcie, uint64_t desc_addr, uint32_t count) {
    if (!pcie || count == 0) return -EINVAL;
    
    /* 查找空闲DMA引擎 */
    for (uint32_t i = 0; i < pcie->num_dma_engines; i++) {
        if (!pcie->dma_engines[i].busy) {
            pcie->dma_engines[i].busy = 1;
            pcie->dma_engines[i].desc_addr = desc_addr;
            pcie->dma_engines[i].desc_count = count;
            pcie->dma_engines[i].bytes_transferred = 0;
            return 0;
        }
    }
    
    return -EBUSY;
}

int pcie_send_interrupt(pcie_controller_t* pcie, uint32_t vector) {
    if (!pcie) return -EINVAL;
    
    if (pcie->host_interrupt) {
        pcie->host_interrupt(vector);
    }
    
    pthread_mutex_lock(&pcie->stats_lock);
    pcie->stats.interrupts_sent++;
    pthread_mutex_unlock(&pcie->stats_lock);
    
    return 0;
}

uint32_t pcie_config_read(pcie_controller_t* pcie, uint16_t offset) {
    if (!pcie || offset >= sizeof(pcie_config_space_t)) return 0xFFFFFFFF;
    
    return *(uint32_t*)((uint8_t*)&pcie->config_space + offset);
}

void pcie_config_write(pcie_controller_t* pcie, uint16_t offset, uint32_t value) {
    if (!pcie || offset >= sizeof(pcie_config_space_t)) return;
    
    /* 处理特殊寄存器 */
    switch (offset) {
        case 0x04:  /* Command */
            pcie->config_space.command = value & 0xFFFF;
            break;
        case 0x10:  /* BAR0 */
            if (value == 0xFFFFFFFF) {
                /* 大小探测 */
                /* 返回BAR大小 */
            } else {
                pcie->config_space.bar[0] = value;
                pcie->bar0_base = value & ~0xF;
            }
            break;
        default:
            *(uint32_t*)((uint8_t*)&pcie->config_space + offset) = value;
            break;
    }
}

uint32_t pcie_mmio_read(pcie_controller_t* pcie, uint64_t addr) {
    if (!pcie) return 0xFFFFFFFF;
    
    /* 模拟MMIO读取 */
    /* 实际实现会根据地址范围返回不同的寄存器值 */
    
    return 0;
}

void pcie_mmio_write(pcie_controller_t* pcie, uint64_t addr, uint32_t value) {
    if (!pcie) return;
    
    /* 模拟MMIO写入 */
    /* 实际实现会根据地址范围处理不同的寄存器 */
}

void pcie_get_stats(pcie_controller_t* pcie, pcie_stats_t* stats) {
    if (!pcie || !stats) return;
    
    pthread_mutex_lock(&pcie->stats_lock);
    memcpy(stats, &pcie->stats, sizeof(pcie_stats_t));
    pthread_mutex_unlock(&pcie->stats_lock);
}

void pcie_dump_info(pcie_controller_t* pcie) {
    if (!pcie) return;
    
    printf("\n[PCIe] Controller Info:\n");
    printf("  Version: Gen%u\n", pcie->config.version);
    printf("  Lanes: x%u\n", pcie->config.lanes);
    printf("  Max Payload: %u bytes\n", pcie->config.max_payload);
    printf("  Max Read Req: %u bytes\n", pcie->config.max_read_req);
    printf("  Vendor ID: 0x%04X\n", pcie->config_space.vendor_id);
    printf("  Device ID: 0x%04X\n", pcie->config_space.device_id);
    
    pcie_stats_t stats;
    pcie_get_stats(pcie, &stats);
    
    printf("\n  Statistics:\n");
    printf("    TLP sent:     %lu\n", stats.tlp_sent);
    printf("    TLP received: %lu\n", stats.tlp_received);
    printf("    DMA bytes read:    %lu MB\n", stats.dma_bytes_read / (1024*1024));
    printf("    DMA bytes written: %lu MB\n", stats.dma_bytes_written / (1024*1024));
    printf("    Interrupts: %lu\n", stats.interrupts_sent);
}

/* DMA线程 */
static void* dma_thread(void* arg) {
    dma_thread_arg_t* thread_arg = (dma_thread_arg_t*)arg;
    pcie_controller_t* pcie = thread_arg->pcie;
    dma_engine_t* dma = thread_arg->dma;
    
    free(thread_arg);
    
    while (dma->running) {
        if (dma->busy && dma->desc_count > 0) {
            /* 处理描述符链 */
            uint64_t desc_ptr = dma->desc_addr;
            uint32_t processed = 0;
            
            while (processed < dma->desc_count && desc_ptr != 0) {
                /* 从主机读取描述符 */
                dma_descriptor_t desc;
                if (pcie->host_dma_read) {
                    pcie->host_dma_read(desc_ptr, &desc, sizeof(desc));
                }
                
                /* 执行DMA传输 */
                execute_dma_transfer(pcie, &desc);
                
                processed++;
                
                /* 检查是否结束 */
                if (desc.flags & DMA_FLAG_EOL) {
                    break;
                }
                
                desc_ptr = desc.next_desc;
            }
            
            /* 发送完成中断 */
            pcie_send_interrupt(pcie, dma->id);
            
            dma->busy = 0;
        }
        
        usleep(10);  /* 10us */
    }
    
    return NULL;
}

static int execute_dma_transfer(pcie_controller_t* pcie, dma_descriptor_t* desc) {
    if (!desc || desc->length == 0) return -EINVAL;
    
    void* buf = malloc(desc->length);
    if (!buf) return -ENOMEM;
    
    /* 确定方向 */
    int is_read = (desc->src_addr < 0x100000000ULL);  /* 简化判断 */
    
    if (is_read) {
        /* Host -> Device */
        if (pcie->host_dma_read) {
            pcie->host_dma_read(desc->src_addr, buf, desc->length);
        }
        /* 写入Device内存 */
        memcpy((void*)desc->dst_addr, buf, desc->length);
        
        pthread_mutex_lock(&pcie->stats_lock);
        pcie->stats.dma_bytes_read += desc->length;
        pthread_mutex_unlock(&pcie->stats_lock);
    } else {
        /* Device -> Host */
        /* 从Device内存读取 */
        memcpy(buf, (void*)desc->src_addr, desc->length);
        
        if (pcie->host_dma_write) {
            pcie->host_dma_write(desc->dst_addr, buf, desc->length);
        }
        
        pthread_mutex_lock(&pcie->stats_lock);
        pcie->stats.dma_bytes_written += desc->length;
        pthread_mutex_unlock(&pcie->stats_lock);
    }
    
    free(buf);
    return 0;
}
