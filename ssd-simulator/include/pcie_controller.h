/**
 * @file pcie_controller.h
 * @brief SSD Simulator - PCIe Controller Header
 */

#ifndef __PCIE_CONTROLLER_H__
#define __PCIE_CONTROLLER_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PCIe配置 */
typedef struct pcie_config {
    uint8_t version;            /* PCIe版本 (3.0, 4.0, 5.0) */
    uint8_t lanes;              /* 通道数 (x1, x4, x8, x16) */
    uint32_t max_payload;       /* Max Payload Size */
    uint32_t max_read_req;      /* Max Read Request Size */
} pcie_config_t;

/* DMA方向 */
typedef enum {
    DMA_DIR_TO_DEVICE = 0,      /* Host -> Device (写入SSD) */
    DMA_DIR_FROM_DEVICE,        /* Device -> Host (从SSD读取) */
} dma_dir_t;

/* DMA描述符 */
typedef struct dma_descriptor {
    uint64_t src_addr;          /* 源地址 */
    uint64_t dst_addr;          /* 目标地址 */
    uint32_t length;            /* 长度 */
    uint32_t flags;             /* 标志 */
    uint64_t next_desc;         /* 下一个描述符地址 */
} __attribute__((packed)) dma_descriptor_t;

#define DMA_FLAG_EOL            (1 << 0)    /* 描述符链结束 */

/* PCIe统计 */
typedef struct pcie_stats {
    uint64_t tlp_sent;
    uint64_t tlp_received;
    uint64_t dma_bytes_read;
    uint64_t dma_bytes_written;
    uint64_t interrupts_sent;
} pcie_stats_t;

/* Opaque类型 */
typedef struct pcie_controller pcie_controller_t;

/* 主机回调函数 */
typedef int (*host_dma_read_fn_t)(uint64_t host_addr, void* buf, size_t len);
typedef int (*host_dma_write_fn_t)(uint64_t host_addr, void* buf, size_t len);
typedef void (*host_interrupt_fn_t)(uint32_t vector);

/**
 * @brief 创建PCIe控制器
 * @param config PCIe配置
 * @return 控制器句柄，NULL失败
 */
pcie_controller_t* pcie_controller_create(pcie_config_t* config);

/**
 * @brief 销毁PCIe控制器
 * @param pcie 控制器句柄
 */
void pcie_controller_destroy(pcie_controller_t* pcie);

/**
 * @brief 注册主机回调函数
 * @param pcie 控制器句柄
 * @param dma_read DMA读取回调
 * @param dma_write DMA写入回调
 * @param interrupt 中断回调
 * @return 0成功
 */
int pcie_register_host_callbacks(pcie_controller_t* pcie,
                                  host_dma_read_fn_t dma_read,
                                  host_dma_write_fn_t dma_write,
                                  host_interrupt_fn_t interrupt);

/**
 * @brief 执行DMA传输
 * @param pcie 控制器句柄
 * @param desc_addr 描述符链表地址
 * @param count 描述符数量
 * @return 0成功，负数错误码
 */
int pcie_dma_submit(pcie_controller_t* pcie, uint64_t desc_addr, uint32_t count);

/**
 * @brief 发送MSI中断
 * @param pcie 控制器句柄
 * @param vector 中断向量
 * @return 0成功
 */
int pcie_send_interrupt(pcie_controller_t* pcie, uint32_t vector);

/**
 * @brief 读取配置空间
 * @param pcie 控制器句柄
 * @param offset 偏移
 * @return 配置值
 */
uint32_t pcie_config_read(pcie_controller_t* pcie, uint16_t offset);

/**
 * @brief 写入配置空间
 * @param pcie 控制器句柄
 * @param offset 偏移
 * @param value 值
 */
void pcie_config_write(pcie_controller_t* pcie, uint16_t offset, uint32_t value);

/**
 * @brief 读取MMIO
 * @param pcie 控制器句柄
 * @param addr 地址
 * @return 值
 */
uint32_t pcie_mmio_read(pcie_controller_t* pcie, uint64_t addr);

/**
 * @brief 写入MMIO
 * @param pcie 控制器句柄
 * @param addr 地址
 * @param value 值
 */
void pcie_mmio_write(pcie_controller_t* pcie, uint64_t addr, uint32_t value);

/**
 * @brief 获取统计信息
 * @param pcie 控制器句柄
 * @param stats 统计输出
 */
void pcie_get_stats(pcie_controller_t* pcie, pcie_stats_t* stats);

/**
 * @brief 打印PCIe信息
 * @param pcie 控制器句柄
 */
void pcie_dump_info(pcie_controller_t* pcie);

#ifdef __cplusplus
}
#endif

#endif /* __PCIE_CONTROLLER_H__ */
