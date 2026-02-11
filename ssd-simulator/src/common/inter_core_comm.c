/**
 * @file inter_core_comm.c
 * @brief SSD Simulator - Inter-Core Communication Implementation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sys/mman.h>
#include "inter_core_comm.h"

/* cpu_relax定义 */
#ifndef cpu_relax
#define cpu_relax() __asm__ volatile("pause" ::: "memory")
#endif

/* ICC管理器 */
struct icc_manager {
    uint32_t local_core_id;
    uint32_t num_cores;
    shared_memory_t* shm;
    
    /* 发送通道数组 (到各核心) */
    icc_channel_t** send_channels;
    
    /* 接收通道数组 (从各核心) */
    icc_channel_t** recv_channels;
    
    /* 消息序列号 */
    uint64_t msg_sequence;
    
    /* 接收缓冲 */
    message_t recv_buffer[32];
    uint32_t recv_count;
    uint32_t recv_idx;
};

/* 线程局部存储 */
static __thread icc_manager_t g_icc = {0};

/* 前向声明 */
static int spsc_ringbuf_init(spsc_ringbuf_t* ring, message_t* buffer, uint32_t capacity);
static int spsc_send(spsc_ringbuf_t* ring, message_t* msg);
static int spsc_recv(spsc_ringbuf_t* ring, message_t* msg);
static uint32_t spsc_recv_batch(spsc_ringbuf_t* ring, message_t* msgs, uint32_t max_count);
static inline uint64_t get_time_ns(void);

shared_memory_t* icc_shm_create(uint32_t num_cores, uint32_t queue_size) {
    if (num_cores == 0 || num_cores > 64) {
        return NULL;
    }
    
    /* 确保queue_size是2的幂 */
    uint32_t capacity = 1;
    while (capacity < queue_size) {
        capacity <<= 1;
    }
    
    /* 计算所需内存 */
    size_t channel_matrix_size = sizeof(icc_channel_t) * num_cores * num_cores;
    size_t buffer_size = sizeof(message_t) * capacity * num_cores * num_cores;
    size_t total_size = sizeof(shared_memory_t) + channel_matrix_size + buffer_size;
    
    /* 分配共享内存 */
    shared_memory_t* shm = calloc(1, total_size);
    if (!shm) {
        return NULL;
    }
    
    shm->num_cores = num_cores;
    shm->size = total_size;
    shm->base_addr = shm;
    
    /* 初始化屏障 */
    pthread_barrier_init(&shm->barrier, NULL, num_cores);
    
    /* 通道矩阵地址 */
    shm->channels = (icc_channel_t*)((uint8_t*)shm + sizeof(shared_memory_t));
    
    /* 缓冲区地址 */
    message_t* buffers = (message_t*)((uint8_t*)shm->channels + channel_matrix_size);
    
    /* 初始化通道 */
    for (uint32_t src = 0; src < num_cores; src++) {
        for (uint32_t dst = 0; dst < num_cores; dst++) {
            if (src == dst) continue;
            
            icc_channel_t* ch = &shm->channels[src * num_cores + dst];
            ch->src_core = src;
            ch->dst_core = dst;
            
            /* 分配ring buffer */
            message_t* buf = &buffers[(src * num_cores + dst) * capacity];
            spsc_ringbuf_init(&ch->send_buf, buf, capacity);
        }
    }
    
    printf("[ICC] Shared memory created: %u cores, queue size %u\n", 
           num_cores, capacity);
    
    return shm;
}

void icc_shm_destroy(shared_memory_t* shm) {
    if (!shm) return;
    
    pthread_barrier_destroy(&shm->barrier);
    free(shm);
    
    printf("[ICC] Shared memory destroyed\n");
}

int icc_init(uint32_t core_id, uint32_t num_cores, shared_memory_t* shm) {
    if (!shm || core_id >= num_cores || num_cores != shm->num_cores) {
        return -EINVAL;
    }
    
    g_icc.local_core_id = core_id;
    g_icc.num_cores = num_cores;
    g_icc.shm = shm;
    g_icc.msg_sequence = core_id * 1000000ULL;
    
    /* 分配通道指针数组 */
    g_icc.send_channels = calloc(num_cores, sizeof(icc_channel_t*));
    g_icc.recv_channels = calloc(num_cores, sizeof(icc_channel_t*));
    
    for (uint32_t i = 0; i < num_cores; i++) {
        if (i == core_id) continue;
        
        /* 发送通道：本核心 -> 核心i */
        g_icc.send_channels[i] = &shm->channels[core_id * num_cores + i];
        
        /* 接收通道：核心i -> 本核心 */
        g_icc.recv_channels[i] = &shm->channels[i * num_cores + core_id];
    }
    
    g_icc.recv_count = 0;
    g_icc.recv_idx = 0;
    
    printf("[ICC] Core %u initialized\n", core_id);
    return 0;
}

void icc_deinit(void) {
    free(g_icc.send_channels);
    free(g_icc.recv_channels);
    memset(&g_icc, 0, sizeof(g_icc));
}

int icc_send(uint32_t dst_core, msg_type_t type, void* payload, uint32_t len) {
    if (dst_core >= g_icc.num_cores || dst_core == g_icc.local_core_id) {
        return -EINVAL;
    }
    if (len > MSG_MAX_PAYLOAD) {
        return -EMSGSIZE;
    }
    
    message_t msg;
    memset(&msg, 0, sizeof(msg));
    
    msg.header.msg_id = ++g_icc.msg_sequence;
    msg.header.type = type;
    msg.header.src_core = g_icc.local_core_id;
    msg.header.dst_core = dst_core;
    msg.header.payload_len = len;
    msg.header.timestamp = get_time_ns();
    
    if (len > 0 && payload) {
        memcpy(msg.payload, payload, len);
    }
    
    icc_channel_t* ch = g_icc.send_channels[dst_core];
    
    int ret = spsc_send(&ch->send_buf, &msg);
    if (ret == 0) {
        ch->msgs_sent++;
        ch->bytes_sent += len;
    } else {
        ch->send_failures++;
    }
    
    return ret;
}

int icc_send_timeout(uint32_t dst_core, msg_type_t type, void* payload, 
                     uint32_t len, uint64_t timeout_ns) {
    uint64_t deadline = get_time_ns() + timeout_ns;
    int ret;
    
    while ((ret = icc_try_send(dst_core, type, payload, len)) != 0) {
        if (ret != -EAGAIN) {
            return ret;
        }
        if (get_time_ns() >= deadline) {
            return -ETIMEDOUT;
        }
        cpu_relax();
    }
    
    return 0;
}

int icc_try_send(uint32_t dst_core, msg_type_t type, void* payload, uint32_t len) {
    if (dst_core >= g_icc.num_cores || dst_core == g_icc.local_core_id) {
        return -EINVAL;
    }
    if (len > MSG_MAX_PAYLOAD) {
        return -EMSGSIZE;
    }
    
    message_t msg;
    memset(&msg, 0, sizeof(msg));
    
    msg.header.msg_id = ++g_icc.msg_sequence;
    msg.header.type = type;
    msg.header.src_core = g_icc.local_core_id;
    msg.header.dst_core = dst_core;
    msg.header.payload_len = len;
    msg.header.timestamp = get_time_ns();
    
    if (len > 0 && payload) {
        memcpy(msg.payload, payload, len);
    }
    
    icc_channel_t* ch = g_icc.send_channels[dst_core];
    return spsc_send(&ch->send_buf, &msg);
}

int icc_recv(message_t* msg) {
    if (!msg) return -EINVAL;
    
    /* 先检查本地缓冲 */
    if (g_icc.recv_idx < g_icc.recv_count) {
        memcpy(msg, &g_icc.recv_buffer[g_icc.recv_idx], sizeof(message_t));
        g_icc.recv_idx++;
        return 0;
    }
    
    /* 批量接收 */
    g_icc.recv_count = 0;
    g_icc.recv_idx = 0;
    
    for (uint32_t src = 0; src < g_icc.num_cores; src++) {
        if (src == g_icc.local_core_id) continue;
        
        icc_channel_t* ch = g_icc.recv_channels[src];
        uint32_t count = spsc_recv_batch(&ch->send_buf, 
                                         &g_icc.recv_buffer[g_icc.recv_count],
                                         32 - g_icc.recv_count);
        
        for (uint32_t i = 0; i < count; i++) {
            ch->msgs_recv++;
        }
        
        g_icc.recv_count += count;
        if (g_icc.recv_count >= 32) break;
    }
    
    if (g_icc.recv_count > 0) {
        memcpy(msg, &g_icc.recv_buffer[0], sizeof(message_t));
        g_icc.recv_idx = 1;
        return 0;
    }
    
    return -EAGAIN;
}

int icc_try_recv(message_t* msg) {
    /* 先检查本地缓冲 */
    if (g_icc.recv_idx < g_icc.recv_count) {
        memcpy(msg, &g_icc.recv_buffer[g_icc.recv_idx], sizeof(message_t));
        g_icc.recv_idx++;
        return 0;
    }
    
    /* 尝试从各通道接收 */
    for (uint32_t src = 0; src < g_icc.num_cores; src++) {
        if (src == g_icc.local_core_id) continue;
        
        icc_channel_t* ch = g_icc.recv_channels[src];
        if (spsc_recv(&ch->send_buf, msg) == 0) {
            ch->msgs_recv++;
            return 0;
        }
    }
    
    return -EAGAIN;
}

uint32_t icc_recv_batch(message_t* msgs, uint32_t max_count) {
    uint32_t received = 0;
    
    for (uint32_t src = 0; src < g_icc.num_cores && received < max_count; src++) {
        if (src == g_icc.local_core_id) continue;
        
        icc_channel_t* ch = g_icc.recv_channels[src];
        uint32_t count = spsc_recv_batch(&ch->send_buf, &msgs[received], 
                                         max_count - received);
        
        for (uint32_t i = 0; i < count; i++) {
            ch->msgs_recv++;
        }
        
        received += count;
    }
    
    return received;
}

uint32_t icc_broadcast(msg_type_t type, void* payload, uint32_t len) {
    uint32_t sent = 0;
    
    for (uint32_t i = 0; i < g_icc.num_cores; i++) {
        if (i == g_icc.local_core_id) continue;
        
        if (icc_send(i, type, payload, len) == 0) {
            sent++;
        }
    }
    
    return sent;
}

int icc_barrier(void) {
    if (!g_icc.shm) return -EINVAL;
    
    int ret = pthread_barrier_wait(&g_icc.shm->barrier);
    return (ret == 0 || ret == PTHREAD_BARRIER_SERIAL_THREAD) ? 0 : -EINVAL;
}

uint32_t icc_get_core_id(void) {
    return g_icc.local_core_id;
}

uint32_t icc_get_num_cores(void) {
    return g_icc.num_cores;
}

void icc_get_stats(uint64_t* msgs_sent, uint64_t* msgs_recv, uint64_t* send_failures) {
    uint64_t sent = 0, recv = 0, failures = 0;
    
    for (uint32_t i = 0; i < g_icc.num_cores; i++) {
        if (i == g_icc.local_core_id) continue;
        
        icc_channel_t* send_ch = g_icc.send_channels[i];
        icc_channel_t* recv_ch = g_icc.recv_channels[i];
        
        sent += send_ch->msgs_sent;
        recv += recv_ch->msgs_recv;
        failures += send_ch->send_failures;
    }
    
    if (msgs_sent) *msgs_sent = sent;
    if (msgs_recv) *msgs_recv = recv;
    if (send_failures) *send_failures = failures;
}

/* SPSC Ring Buffer实现 */
static int spsc_ringbuf_init(spsc_ringbuf_t* ring, message_t* buffer, uint32_t capacity) {
    ring->head = 0;
    ring->tail = 0;
    ring->capacity = capacity;
    ring->mask = capacity - 1;
    ring->buffer = buffer;
    
    /* 清零缓冲区 */
    memset(buffer, 0, sizeof(message_t) * capacity);
    
    return 0;
}

static int spsc_send(spsc_ringbuf_t* ring, message_t* msg) {
    uint64_t head = ring->head;
    uint64_t next = (head + 1) & ring->mask;
    
    /* 检查满 */
    if (next == ring->tail) {
        return -EAGAIN;
    }
    
    /* 复制消息 */
    memcpy(&ring->buffer[head], msg, sizeof(message_t));
    
    /* 内存屏障 */
    compiler_barrier();
    
    /* 更新head */
    ring->head = next;
    
    return 0;
}

static int spsc_recv(spsc_ringbuf_t* ring, message_t* msg) {
    uint64_t tail = ring->tail;
    
    /* 检查空 */
    if (tail == ring->head) {
        return -EAGAIN;
    }
    
    /* 复制消息 */
    memcpy(msg, &ring->buffer[tail], sizeof(message_t));
    
    /* 内存屏障 */
    compiler_barrier();
    
    /* 更新tail */
    ring->tail = (tail + 1) & ring->mask;
    
    return 0;
}

static uint32_t spsc_recv_batch(spsc_ringbuf_t* ring, message_t* msgs, uint32_t max_count) {
    uint64_t tail = ring->tail;
    uint64_t head = ring->head;
    uint32_t count = 0;
    
    while (tail != head && count < max_count) {
        memcpy(&msgs[count], &ring->buffer[tail], sizeof(message_t));
        tail = (tail + 1) & ring->mask;
        count++;
    }
    
    if (count > 0) {
        compiler_barrier();
        ring->tail = tail;
    }
    
    return count;
}

static inline uint64_t get_time_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

/* cpu_relax定义 */
#define cpu_relax() __asm__ volatile("pause" ::: "memory")
