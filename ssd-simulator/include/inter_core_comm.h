/**
 * @file inter_core_comm.h
 * @brief SSD Simulator - Inter-Core Communication Header
 */

#ifndef __INTER_CORE_COMM_H__
#define __INTER_CORE_COMM_H__

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 消息类型 */
typedef enum {
    MSG_TYPE_CMD = 0,           /* 命令消息 */
    MSG_TYPE_DATA,              /* 数据消息 */
    MSG_TYPE_EVENT,             /* 事件通知 */
    MSG_TYPE_SYNC,              /* 同步消息 */
    MSG_TYPE_NUM
} msg_type_t;

/* 最大负载大小 */
#define MSG_MAX_PAYLOAD     256

/* 消息头 */
typedef struct msg_header {
    uint64_t msg_id;            /* 消息ID */
    msg_type_t type;            /* 类型 */
    uint32_t src_core;          /* 源核心 */
    uint32_t dst_core;          /* 目标核心 */
    uint32_t payload_len;       /* 负载长度 */
    uint64_t timestamp;         /* 时间戳 */
} msg_header_t;

/* 消息结构 */
typedef struct message {
    msg_header_t header;
    uint8_t payload[MSG_MAX_PAYLOAD];
    volatile uint32_t ready;    /* 就绪标志 (for ring buffer) */
    char _pad[60];              /* 缓存行对齐到128字节 */
} __attribute__((aligned(128))) message_t;

/* SPSC Ring Buffer */
typedef struct spsc_ringbuf {
    volatile uint64_t head __attribute__((aligned(64)));     /* 写指针 */
    volatile uint64_t tail __attribute__((aligned(64)));     /* 读指针 */
    uint32_t capacity;
    uint32_t mask;
    message_t* buffer;
    char _pad[64 - 2*sizeof(uint64_t) - 2*sizeof(uint32_t) - sizeof(void*)];
} __attribute__((aligned(64))) spsc_ringbuf_t;

/* 核间通道 */
typedef struct icc_channel {
    uint32_t src_core;
    uint32_t dst_core;
    
    spsc_ringbuf_t send_buf;    /* 发送缓冲区 */
    
    /* 统计 */
    uint64_t msgs_sent;
    uint64_t msgs_recv;
    uint64_t bytes_sent;
    uint64_t send_failures;
    char _pad[64 - 4*sizeof(uint64_t)];
} __attribute__((aligned(64))) icc_channel_t;

/* 共享内存区域 */
typedef struct shared_memory {
    void* base_addr;
    size_t size;
    uint32_t num_cores;
    
    /* 通道矩阵 */
    icc_channel_t* channels;    /* 大小: num_cores * num_cores */
    
    /* 全局屏障 */
    pthread_barrier_t barrier;
} shared_memory_t;

/* ICC管理器 (opaque) */
typedef struct icc_manager icc_manager_t;

/* 消息处理回调 */
typedef void (*msg_handler_t)(message_t* msg, void* user_data);

/* API函数 */

/**
 * @brief 初始化核间通信
 * @param core_id 本地核心ID
 * @param num_cores 总核心数
 * @param shm 共享内存区域
 * @return 0成功，负数错误码
 */
int icc_init(uint32_t core_id, uint32_t num_cores, shared_memory_t* shm);

/**
 * @brief 销毁核间通信
 */
void icc_deinit(void);

/**
 * @brief 创建共享内存区域
 * @param num_cores 核心数
 * @param queue_size 每队列大小
 * @return 共享内存指针，NULL失败
 */
shared_memory_t* icc_shm_create(uint32_t num_cores, uint32_t queue_size);

/**
 * @brief 销毁共享内存区域
 * @param shm 共享内存指针
 */
void icc_shm_destroy(shared_memory_t* shm);

/**
 * @brief 发送消息
 * @param dst_core 目标核心
 * @param type 消息类型
 * @param payload 负载数据
 * @param len 负载长度
 * @return 0成功，负数错误码
 */
int icc_send(uint32_t dst_core, msg_type_t type, void* payload, uint32_t len);

/**
 * @brief 发送消息（带超时）
 * @param dst_core 目标核心
 * @param type 消息类型
 * @param payload 负载数据
 * @param len 负载长度
 * @param timeout_ns 超时时间（纳秒）
 * @return 0成功，负数错误码
 */
int icc_send_timeout(uint32_t dst_core, msg_type_t type, void* payload, 
                     uint32_t len, uint64_t timeout_ns);

/**
 * @brief 尝试发送消息（非阻塞）
 * @param dst_core 目标核心
 * @param type 消息类型
 * @param payload 负载数据
 * @param len 负载长度
 * @return 0成功，-EAGAIN缓冲区满，其他负数错误码
 */
int icc_try_send(uint32_t dst_core, msg_type_t type, void* payload, uint32_t len);

/**
 * @brief 接收消息
 * @param msg 消息输出
 * @return 0成功，负数错误码
 */
int icc_recv(message_t* msg);

/**
 * @brief 尝试接收消息（非阻塞）
 * @param msg 消息输出
 * @return 0成功，-EAGAIN缓冲区空，其他负数错误码
 */
int icc_try_recv(message_t* msg);

/**
 * @brief 批量接收消息
 * @param msgs 消息数组
 * @param max_count 最大接收数量
 * @return 实际接收数量
 */
uint32_t icc_recv_batch(message_t* msgs, uint32_t max_count);

/**
 * @brief 广播消息到所有核心
 * @param type 消息类型
 * @param payload 负载数据
 * @param len 负载长度
 * @return 成功发送的核心数
 */
uint32_t icc_broadcast(msg_type_t type, void* payload, uint32_t len);

/**
 * @brief 全局屏障同步
 * @return 0成功，负数错误码
 */
int icc_barrier(void);

/**
 * @brief 获取本地核心ID
 * @return 核心ID
 */
uint32_t icc_get_core_id(void);

/**
 * @brief 获取核心数
 * @return 核心数
 */
uint32_t icc_get_num_cores(void);

/**
 * @brief 获取统计信息
 * @param msgs_sent 发送消息数输出
 * @param msgs_recv 接收消息数输出
 * @param send_failures 发送失败数输出
 */
void icc_get_stats(uint64_t* msgs_sent, uint64_t* msgs_recv, uint64_t* send_failures);

/* 内存屏障 */
#define memory_barrier() __sync_synchronize()
#define compiler_barrier() __asm__ volatile("" ::: "memory")

#ifdef __cplusplus
}
#endif

#endif /* __INTER_CORE_COMM_H__ */
