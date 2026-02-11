# SSD 模拟器 - 通用模块设计文档

## 目录
1. [任务调度器 (TaskScheduler)](#1-任务调度器-taskscheduler)
2. [核间通信 (InterCoreComm)](#2-核间通信-intercorecomm)
3. [内存管理 (DDRManager)](#3-内存管理-ddrmanager)
4. [元数据管理 (MetadataManager)](#4-元数据管理-metadatamanager)

---

## 1. 任务调度器 (TaskScheduler)

### 1.1 设计目标
- 支持多优先级任务调度
- 支持抢占式和非抢占式任务
- Work Stealing实现负载均衡
- 低延迟任务分发

### 1.2 数据结构

```c
// 任务类型枚举
typedef enum {
    TASK_TYPE_IO_READ,      // IO读任务
    TASK_TYPE_IO_WRITE,     // IO写任务
    TASK_TYPE_IO_FLUSH,     // Flush任务
    TASK_TYPE_GC,           // 垃圾回收
    TASK_TYPE_WL,           // 磨损均衡
    TASK_TYPE_ADMIN,        // 管理任务
    TASK_TYPE_BG,           // 后台任务
    TASK_TYPE_NUM
} task_type_t;

// 任务优先级
typedef enum {
    TASK_PRIO_HIGHEST = 0,  // 最高优先级 (Admin, 中断处理)
    TASK_PRIO_HIGH,         // 高优先级 (IO读)
    TASK_PRIO_NORMAL,       // 正常优先级 (IO写)
    TASK_PRIO_LOW,          // 低优先级 (GC)
    TASK_PRIO_LOWEST,       // 最低优先级 (后台监控)
    TASK_PRIO_NUM
} task_prio_t;

// 任务状态
typedef enum {
    TASK_STATE_PENDING,     // 等待中
    TASK_STATE_RUNNING,     // 运行中
    TASK_STATE_BLOCKED,     // 阻塞
    TASK_STATE_COMPLETED,   // 完成
    TASK_STATE_CANCELLED    // 取消
} task_state_t;

// 任务上下文
typedef struct task_ctx {
    uint64_t task_id;               // 任务ID
    task_type_t type;               // 任务类型
    task_prio_t priority;           // 优先级
    task_state_t state;             // 状态
    
    uint32_t core_id;               // 目标核心(绑定)
    uint32_t src_core;              // 来源核心
    
    void (*execute)(struct task_ctx*);  // 执行函数
    void (*complete)(struct task_ctx*); // 完成回调
    void* private_data;             // 私有数据
    
    uint64_t submit_time;           // 提交时间
    uint64_t start_time;            // 开始时间
    uint64_t complete_time;         // 完成时间
    
    struct task_ctx* next;          // 链表指针
    struct task_ctx* prev;
} task_ctx_t;

// 任务队列 (Per Core)
typedef struct task_queue {
    task_ctx_t* heads[TASK_PRIO_NUM];   // 各优先级队头
    task_ctx_t* tails[TASK_PRIO_NUM];   // 各优先级队尾
    uint32_t counts[TASK_PRIO_NUM];     // 各优先级计数
    uint32_t total_count;               // 总任务数
    
    pthread_spinlock_t lock;            // 自旋锁保护
} task_queue_t;

// 调度器实例 (Per Core)
typedef struct task_scheduler {
    uint32_t core_id;                   // 核心ID
    task_queue_t local_queue;           // 本地任务队列
    
    // Work Stealing
    struct task_scheduler** all_scheds; // 所有调度器数组
    uint32_t num_cores;                 // 总核心数
    uint32_t steal_attempts;            // 偷取尝试次数
    
    // 执行线程
    pthread_t worker_thread;
    volatile int running;               // 运行标志
    
    // 统计
    uint64_t tasks_executed;
    uint64_t tasks_stolen;
    uint64_t total_latency_ns;
} task_scheduler_t;

// 全局调度器
typedef struct global_scheduler {
    task_scheduler_t** schedulers;      // 各核心调度器
    uint32_t num_cores;                 // 核心数
    uint64_t next_task_id;              // 任务ID计数器
    pthread_mutex_t id_lock;            // ID锁
    
    // 负载均衡
    uint32_t rr_counter;                // 轮询计数器
} global_scheduler_t;
```

### 1.3 核心算法

#### 1.3.1 任务提交
```c
int task_submit(task_ctx_t* task, uint32_t preferred_core) {
    // 1. 分配任务ID
    task->task_id = atomic_fetch_add(&g_sched.next_task_id, 1);
    task->submit_time = get_time_ns();
    task->state = TASK_STATE_PENDING;
    
    // 2. 选择目标核心
    uint32_t target_core = preferred_core;
    if (target_core >= g_sched.num_cores) {
        target_core = atomic_fetch_add(&g_sched.rr_counter, 1) % g_sched.num_cores;
    }
    
    // 3. 加入目标核心队列
    task_queue_t* queue = &g_sched.schedulers[target_core]->local_queue;
    
    pthread_spin_lock(&queue->lock);
    // 按优先级插入队尾
    task->next = NULL;
    if (queue->tails[task->priority] == NULL) {
        queue->heads[task->priority] = task;
    } else {
        queue->tails[task->priority]->next = task;
    }
    queue->tails[task->priority] = task;
    queue->counts[task->priority]++;
    queue->total_count++;
    pthread_spin_unlock(&queue->lock);
    
    return 0;
}
```

#### 1.3.2 任务调度 (Work Stealing)
```c
task_ctx_t* task_schedule(task_scheduler_t* sched) {
    task_queue_t* queue = &sched->local_queue;
    task_ctx_t* task = NULL;
    
    // 1. 尝试从本地队列获取任务 (优先级从高到低)
    pthread_spin_lock(&queue->lock);
    for (int prio = TASK_PRIO_HIGHEST; prio < TASK_PRIO_NUM; prio++) {
        if (queue->heads[prio] != NULL) {
            task = queue->heads[prio];
            queue->heads[prio] = task->next;
            if (queue->heads[prio] == NULL) {
                queue->tails[prio] = NULL;
            }
            queue->counts[prio]--;
            queue->total_count--;
            task->next = NULL;
            break;
        }
    }
    pthread_spin_unlock(&queue->lock);
    
    // 2. 本地无任务，尝试Work Stealing
    if (task == NULL) {
        task = task_steal(sched);
    }
    
    if (task) {
        task->state = TASK_STATE_RUNNING;
        task->start_time = get_time_ns();
    }
    
    return task;
}

task_ctx_t* task_steal(task_scheduler_t* sched) {
    // 随机选择 victim core
    uint32_t victim = (sched->core_id + 1 + rand() % (sched->num_cores - 1)) % sched->num_cores;
    task_scheduler_t* victim_sched = sched->all_scheds[victim];
    task_queue_t* victim_queue = &victim_sched->local_queue;
    
    // 从victim的最低优先级队列偷取 (减少干扰)
    task_ctx_t* task = NULL;
    
    if (pthread_spin_trylock(&victim_queue->lock) == 0) {
        for (int prio = TASK_PRIO_LOWEST; prio >= TASK_PRIO_NORMAL; prio--) {
            if (victim_queue->counts[prio] > 1) {  // 保留至少一个
                // 从队尾偷取 (LIFO，更好的局部性)
                task = victim_queue->tails[prio];
                if (task && task->prev) {
                    victim_queue->tails[prio] = task->prev;
                    task->prev->next = NULL;
                    task->prev = NULL;
                    victim_queue->counts[prio]--;
                    victim_queue->total_count--;
                    sched->tasks_stolen++;
                }
                break;
            }
        }
        pthread_spin_unlock(&victim_queue->lock);
    }
    
    return task;
}
```

#### 1.3.3 任务执行循环
```c
void* worker_thread(void* arg) {
    task_scheduler_t* sched = (task_scheduler_t*)arg;
    
    while (sched->running) {
        task_ctx_t* task = task_schedule(sched);
        
        if (task) {
            // 执行任务
            task->execute(task);
            
            // 更新统计
            task->complete_time = get_time_ns();
            sched->tasks_executed++;
            sched->total_latency_ns += (task->complete_time - task->submit_time);
            
            // 完成回调
            if (task->complete) {
                task->complete(task);
            }
            
            task->state = TASK_STATE_COMPLETED;
        } else {
            // 无任务，短暂休眠或spin
            cpu_relax();
        }
    }
    
    return NULL;
}
```

### 1.4 API接口

```c
// 初始化/销毁
int task_scheduler_init(uint32_t num_cores);
void task_scheduler_destroy(void);

// 任务操作
int task_submit(task_ctx_t* task, uint32_t preferred_core);
int task_submit_batch(task_ctx_t** tasks, uint32_t count, uint32_t preferred_core);
int task_cancel(uint64_t task_id);

// 查询
uint32_t task_queue_depth(uint32_t core_id);
uint64_t task_get_latency_stats(uint32_t core_id, double* avg_latency);
```

---

## 2. 核间通信 (InterCoreComm)

### 2.1 设计目标
- 低延迟核间消息传递
- 支持多种通信模式：消息队列、共享内存、信号量
- Lock-Free实现，避免缓存行乒乓
- 支持批量消息处理

### 2.2 数据结构

```c
// 消息类型
typedef enum {
    MSG_TYPE_CMD,           // 命令消息
    MSG_TYPE_DATA,          // 数据消息
    MSG_TYPE_EVENT,         // 事件通知
    MSG_TYPE_SYNC,          // 同步消息
    MSG_TYPE_NUM
} msg_type_t;

// 消息头
typedef struct msg_header {
    uint64_t msg_id;        // 消息ID
    msg_type_t type;        // 类型
    uint32_t src_core;      // 源核心
    uint32_t dst_core;      // 目标核心
    uint32_t payload_len;   // 负载长度
    uint64_t timestamp;     // 时间戳
} msg_header_t;

// 消息结构
typedef struct message {
    msg_header_t header;
    uint8_t payload[MSG_MAX_PAYLOAD];  // 最大负载
    volatile uint32_t ready;            // 就绪标志
} message_t;

// Lock-Free Ring Buffer (单生产者单消费者)
typedef struct spsc_ringbuf {
    volatile uint64_t head;     // 写指针
    volatile uint64_t tail;     // 读指针
    uint32_t capacity;          // 容量(2的幂)
    uint32_t mask;              // 掩码
    message_t* buffer;          // 缓冲区
    char _pad[64 - 3*sizeof(uint32_t) - sizeof(void*)];  // 缓存行对齐
} spsc_ringbuf_t;

// 多生产者单消费者 Ring Buffer
typedef struct mpsc_ringbuf {
    volatile uint64_t head;         // 写指针
    volatile uint64_t tail;         // 读指针
    volatile uint64_t commit_head;  // 提交指针
    uint32_t capacity;
    uint32_t mask;
    message_t* buffer;
    pthread_spinlock_t lock;        // 仅用于多生产者
    char _pad[64];
} mpsc_ringbuf_t;

// 核间通道 (每对核心一个)
typedef struct icc_channel {
    uint32_t src_core;          // 源核心
    uint32_t dst_core;          // 目标核心
    
    spsc_ringbuf_t send_buf;    // 发送缓冲区
    spsc_ringbuf_t recv_buf;    // 接收缓冲区 (对端视角)
    
    // 统计
    uint64_t msgs_sent;
    uint64_t msgs_recv;
    uint64_t bytes_sent;
    uint64_t send_failures;
} icc_channel_t;

// 共享内存区域
typedef struct shared_memory {
    void* base_addr;            // 基地址
    size_t size;                // 总大小
    uint32_t num_cores;         // 核心数
    
    // 通道矩阵 (num_cores x num_cores)
    icc_channel_t** channels;
    
    // 全局屏障
    pthread_barrier_t barrier;
    
    // 原子变量区
    volatile uint64_t* atomic_vars;
} shared_memory_t;

// 核间通信管理器
typedef struct icc_manager {
    uint32_t local_core_id;     // 本地核心ID
    uint32_t num_cores;         // 总核心数
    shared_memory_t* shm;       // 共享内存
    
    // 本核心相关的通道
    icc_channel_t** send_channels;   // 发送到其他核心
    icc_channel_t** recv_channels;   // 从其他核心接收
    
    // 接收线程
    pthread_t recv_thread;
    void (*msg_handler)(message_t* msg);
    volatile int running;
} icc_manager_t;
```

### 2.3 核心算法

#### 2.3.1 Lock-Free SPSC Ring Buffer
```c
// 发送消息 (生产者)
int spsc_send(spsc_ringbuf_t* ring, message_t* msg) {
    uint64_t head = ring->head;
    uint64_t next = (head + 1) & ring->mask;
    
    // 检查满
    if (next == ring->tail) {
        return -EAGAIN;  // 缓冲区满
    }
    
    // 写入数据
    memcpy(&ring->buffer[head], msg, sizeof(message_t));
    ring->buffer[head].ready = 1;
    
    // 内存屏障 + 更新head
    memory_barrier();
    ring->head = next;
    
    return 0;
}

// 接收消息 (消费者)
int spsc_recv(spsc_ringbuf_t* ring, message_t* msg) {
    uint64_t tail = ring->tail;
    
    // 检查空
    if (tail == ring->head) {
        return -EAGAIN;  // 缓冲区空
    }
    
    // 检查就绪标志
    if (!ring->buffer[tail].ready) {
        return -EAGAIN;  // 数据未就绪
    }
    
    // 读取数据
    memcpy(msg, &ring->buffer[tail], sizeof(message_t));
    ring->buffer[tail].ready = 0;
    
    // 内存屏障 + 更新tail
    memory_barrier();
    ring->tail = (tail + 1) & ring->mask;
    
    return 0;
}

// 批量接收 (减少缓存同步开销)
int spsc_recv_batch(spsc_ringbuf_t* ring, message_t* msgs, uint32_t max_count) {
    uint64_t tail = ring->tail;
    uint64_t head = ring->head;
    uint32_t count = 0;
    
    while (tail != head && count < max_count) {
        if (!ring->buffer[tail].ready) break;
        
        memcpy(&msgs[count], &ring->buffer[tail], sizeof(message_t));
        ring->buffer[tail].ready = 0;
        tail = (tail + 1) & ring->mask;
        count++;
    }
    
    if (count > 0) {
        memory_barrier();
        ring->tail = tail;
    }
    
    return count;
}
```

#### 2.3.2 核间消息发送
```c
int icc_send(uint32_t dst_core, msg_type_t type, void* payload, uint32_t len) {
    icc_manager_t* icc = get_icc_manager();
    
    if (dst_core >= icc->num_cores || dst_core == icc->local_core_id) {
        return -EINVAL;
    }
    
    icc_channel_t* ch = icc->send_channels[dst_core];
    
    message_t msg;
    msg.header.msg_id = atomic_fetch_add(&msg_id_counter, 1);
    msg.header.type = type;
    msg.header.src_core = icc->local_core_id;
    msg.header.dst_core = dst_core;
    msg.header.payload_len = len;
    msg.header.timestamp = get_time_ns();
    
    if (len > MSG_MAX_PAYLOAD) {
        len = MSG_MAX_PAYLOAD;
    }
    memcpy(msg.payload, payload, len);
    msg.ready = 0;
    
    int ret = spsc_send(&ch->send_buf, &msg);
    if (ret == 0) {
        ch->msgs_sent++;
        ch->bytes_sent += len;
    } else {
        ch->send_failures++;
    }
    
    return ret;
}

// 带超时的发送
int icc_send_timeout(uint32_t dst_core, msg_type_t type, void* payload, 
                     uint32_t len, uint64_t timeout_ns) {
    uint64_t deadline = get_time_ns() + timeout_ns;
    int ret;
    
    while ((ret = icc_send(dst_core, type, payload, len)) != 0) {
        if (get_time_ns() >= deadline) {
            return -ETIMEDOUT;
        }
        cpu_relax();
    }
    
    return 0;
}
```

#### 2.3.3 广播与汇聚
```c
// 广播到所有核心 (除自己)
int icc_broadcast(msg_type_t type, void* payload, uint32_t len, uint64_t* failed_cores) {
    icc_manager_t* icc = get_icc_manager();
    uint64_t failed = 0;
    
    for (uint32_t i = 0; i < icc->num_cores; i++) {
        if (i == icc->local_core_id) continue;
        
        int ret = icc_send(i, type, payload, len);
        if (ret != 0) {
            failed |= (1ULL << i);
        }
    }
    
    if (failed_cores) *failed_cores = failed;
    return (failed == 0) ? 0 : -EAGAIN;
}

// 汇聚等待 (等待所有核心的响应)
int icc_gather(uint32_t expected_responses, message_t* responses, 
               uint64_t timeout_ns) {
    uint64_t deadline = get_time_ns() + timeout_ns;
    uint32_t received = 0;
    
    while (received < expected_responses) {
        for (uint32_t i = 0; i < get_num_cores(); i++) {
            if (i == get_core_id()) continue;
            
            message_t msg;
            if (icc_try_recv(i, &msg) == 0) {
                memcpy(&responses[received], &msg, sizeof(message_t));
                received++;
            }
        }
        
        if (get_time_ns() >= deadline) {
            return -ETIMEDOUT;
        }
        
        if (received < expected_responses) {
            cpu_relax();
        }
    }
    
    return 0;
}
```

### 2.4 共享内存管理
```c
// 创建共享内存区域
shared_memory_t* shared_memory_create(uint32_t num_cores, size_t shm_size) {
    shared_memory_t* shm = calloc(1, sizeof(shared_memory_t));
    
    shm->num_cores = num_cores;
    shm->size = shm_size;
    
    // 使用 huge pages 分配大页内存
    shm->base_addr = mmap(NULL, shm_size, PROT_READ | PROT_WRITE,
                          MAP_SHARED | MAP_ANONYMOUS | MAP_HUGETLB, -1, 0);
    if (shm->base_addr == MAP_FAILED) {
        // fallback to normal pages
        shm->base_addr = mmap(NULL, shm_size, PROT_READ | PROT_WRITE,
                              MAP_SHARED | MAP_ANONYMOUS, -1, 0);
    }
    
    // 初始化屏障
    pthread_barrier_init(&shm->barrier, NULL, num_cores);
    
    // 分配通道矩阵
    size_t channel_size = sizeof(icc_channel_t) * num_cores * num_cores;
    shm->channels = (icc_channel_t**)shm->base_addr;
    
    // 初始化通道
    for (uint32_t i = 0; i < num_cores; i++) {
        for (uint32_t j = 0; j < num_cores; j++) {
            if (i == j) continue;
            
            icc_channel_t* ch = &shm->channels[i][j];
            ch->src_core = i;
            ch->dst_core = j;
            
            // 分配Ring Buffer
            spsc_ringbuf_init(&ch->send_buf, MSG_QUEUE_SIZE);
        }
    }
    
    return shm;
}
```

### 2.5 API接口
```c
// 初始化
int icc_init(uint32_t core_id, uint32_t num_cores, shared_memory_t* shm);
void icc_deinit(void);

// 基本通信
int icc_send(uint32_t dst_core, msg_type_t type, void* payload, uint32_t len);
int icc_send_timeout(uint32_t dst_core, msg_type_t type, void* payload, 
                     uint32_t len, uint64_t timeout_ns);
int icc_recv(message_t* msg);
int icc_try_recv(uint32_t src_core, message_t* msg);

// 高级通信
int icc_broadcast(msg_type_t type, void* payload, uint32_t len, uint64_t* failed);
int icc_gather(uint32_t expected, message_t* responses, uint64_t timeout_ns);
int icc_barrier(void);

// 共享内存
void* icc_shm_alloc(size_t size);
void icc_shm_free(void* ptr);
```

---

## 3. 内存管理 (DDRManager)

### 3.1 设计目标
- 高效的内存分配/释放
- 支持多种内存类型：数据缓冲区、元数据缓冲区、映射表缓存
- 内存对齐管理
- 内存池化减少碎片

### 3.2 数据结构

```c
// 内存块大小等级
typedef enum {
    MEM_BLOCK_4K = 0,       // 4KB   - 小IO, 元数据
    MEM_BLOCK_16K,          // 16KB  - NAND页大小
    MEM_BLOCK_64K,          // 64KB  - 大IO聚合
    MEM_BLOCK_256K,         // 256KB - 映射表块
    MEM_BLOCK_1M,           // 1MB   - 大缓冲区
    MEM_BLOCK_4M,           // 4MB   - 超大IO
    MEM_BLOCK_NUM
} mem_block_size_t;

#define MEM_BLOCK_SIZES {4096, 16384, 65536, 262144, 1048576, 4194304}

// 内存块头
typedef struct mem_block {
    uint32_t magic;             // 魔数 (用于校验)
    mem_block_size_t size_class; // 大小等级
    uint32_t flags;             // 标志位
    uint32_t ref_count;         // 引用计数
    
    union {
        struct {                // 已分配
            uint64_t alloc_time;
            const char* alloc_file;
            uint32_t alloc_line;
        } alloc_info;
        
        struct {                // 空闲链表
            struct mem_block* next;
            struct mem_block* prev;
        } free_list;
    };
    
    uint8_t data[];             // 实际数据 (柔性数组)
} mem_block_t;

// Slab分配器 (Per Size Class)
typedef struct slab_allocator {
    mem_block_size_t size_class;
    size_t block_size;
    size_t block_count;
    
    void* memory_pool;          // 预分配的内存池
    mem_block_t* free_list;     // 空闲链表
    uint32_t free_count;        // 空闲数量
    uint32_t used_count;        // 已用数量
    
    pthread_spinlock_t lock;    // 保护锁
    
    // 统计
    uint64_t allocs;
    uint64_t frees;
    uint64_t alloc_failures;
} slab_allocator_t;

// 内存缓冲区 (用于IO数据)
typedef struct data_buffer {
    void* virtual_addr;         // 虚拟地址
    uint64_t physical_addr;     // 模拟物理地址
    size_t size;                // 大小
    uint32_t align;             // 对齐要求
    
    // 链表
    struct data_buffer* next;
    struct data_buffer* prev;
} data_buffer_t;

// 缓冲区池
typedef struct buffer_pool {
    const char* name;           // 池名称
    size_t buffer_size;         // 缓冲区大小
    uint32_t num_buffers;       // 缓冲区数量
    
    data_buffer_t* buffers;     // 缓冲区数组
    data_buffer_t* free_list;   // 空闲链表
    uint32_t free_count;
    
    pthread_spinlock_t lock;
    
    // 统计
    uint64_t hits;              // 命中次数
    uint64_t misses;            // 未命中次数
} buffer_pool_t;

// DDR内存管理器
typedef struct ddr_manager {
    // 配置
    uint64_t total_capacity;    // 总容量
    uint64_t used_capacity;     // 已用容量
    
    // Slab分配器 (每种大小等级一个)
    slab_allocator_t slabs[MEM_BLOCK_NUM];
    
    // 专用缓冲区池
    buffer_pool_t data_pool;    // 数据缓冲区池
    buffer_pool_t meta_pool;    // 元数据缓冲区池
    buffer_pool_t map_pool;     // 映射表缓冲区池
    
    // 大内存分配器 (大于4MB)
    void* big_mem_start;
    void* big_mem_end;
    void* big_mem_current;
    
    // 统计
    uint64_t total_allocs;
    uint64_t total_frees;
    uint64_t peak_usage;
} ddr_manager_t;
```

### 3.3 核心算法

#### 3.3.1 Slab分配
```c
void* slab_alloc(slab_allocator_t* slab, const char* file, uint32_t line) {
    pthread_spin_lock(&slab->lock);
    
    mem_block_t* block = slab->free_list;
    if (block) {
        // 从空闲链表取出
        slab->free_list = block->free_list.next;
        if (slab->free_list) {
            slab->free_list->free_list.prev = NULL;
        }
        slab->free_count--;
        slab->used_count++;
        pthread_spin_unlock(&slab->lock);
        
        // 初始化块头
        block->magic = MEM_BLOCK_MAGIC;
        block->ref_count = 1;
        block->flags = 0;
        block->alloc_info.alloc_time = get_time_ns();
        block->alloc_info.alloc_file = file;
        block->alloc_info.alloc_line = line;
        
        slab->allocs++;
        return block->data;
    }
    
    pthread_spin_unlock(&slab->lock);
    slab->alloc_failures++;
    return NULL;  // 内存不足
}

void slab_free(slab_allocator_t* slab, void* ptr) {
    if (!ptr) return;
    
    mem_block_t* block = (mem_block_t*)((uint8_t*)ptr - sizeof(mem_block_t));
    
    // 校验
    if (block->magic != MEM_BLOCK_MAGIC) {
        log_error("Invalid memory block freed!");
        return;
    }
    
    // 引用计数管理
    uint32_t ref = atomic_fetch_sub(&block->ref_count, 1);
    if (ref > 1) {
        return;  // 还有引用
    }
    
    // 放回空闲链表
    pthread_spin_lock(&slab->lock);
    block->free_list.next = slab->free_list;
    block->free_list.prev = NULL;
    if (slab->free_list) {
        slab->free_list->free_list.prev = block;
    }
    slab->free_list = block;
    slab->free_count++;
    slab->used_count--;
    pthread_spin_unlock(&slab->lock);
    
    slab->frees++;
}
```

#### 3.3.2 缓冲区池管理
```c
data_buffer_t* buffer_pool_acquire(buffer_pool_t* pool) {
    pthread_spin_lock(&pool->lock);
    
    data_buffer_t* buf = pool->free_list;
    if (buf) {
        pool->free_list = buf->next;
        if (pool->free_list) {
            pool->free_list->prev = NULL;
        }
        pool->free_count--;
        pthread_spin_unlock(&pool->lock);
        
        buf->next = NULL;
        buf->prev = NULL;
        pool->hits++;
        return buf;
    }
    
    pthread_spin_unlock(&pool->lock);
    
    // 池耗尽，从slab分配
    pool->misses++;
    return NULL;
}

void buffer_pool_release(buffer_pool_t* pool, data_buffer_t* buf) {
    if (!buf) return;
    
    pthread_spin_lock(&pool->lock);
    buf->next = pool->free_list;
    buf->prev = NULL;
    if (pool->free_list) {
        pool->free_list->prev = buf;
    }
    pool->free_list = buf;
    pool->free_count++;
    pthread_spin_unlock(&pool->lock);
}
```

#### 3.3.3 DMA地址转换
```c
// 虚拟地址到物理地址 (模拟)
uint64_t ddr_virt_to_phys(void* virt_addr) {
    ddr_manager_t* ddr = get_ddr_manager();
    
    // 简单线性映射
    uint64_t offset = (uint64_t)virt_addr - (uint64_t)ddr->base_addr;
    return ddr->phys_base + offset;
}

// 物理地址到虚拟地址
void* ddr_phys_to_virt(uint64_t phys_addr) {
    ddr_manager_t* ddr = get_ddr_manager();
    
    uint64_t offset = phys_addr - ddr->phys_base;
    return (void*)((uint64_t)ddr->base_addr + offset);
}

// 获取物理上连续的缓冲区 (模拟)
void* ddr_alloc_contiguous(size_t size, uint32_t align) {
    // 对齐到指定边界
    size_t aligned_size = ALIGN_UP(size, align);
    
    // 从合适的slab分配
    for (int i = 0; i < MEM_BLOCK_NUM; i++) {
        if (g_block_sizes[i] >= aligned_size) {
            return slab_alloc(&ddr->slabs[i], __FILE__, __LINE__);
        }
    }
    
    // 超大内存直接分配
    return ddr_alloc_big(aligned_size);
}
```

### 3.4 API接口
```c
// 初始化
int ddr_manager_init(uint64_t capacity);
void ddr_manager_destroy(void);

// 内存分配
void* ddr_alloc(size_t size, uint32_t align);
void* ddr_alloc_tracked(size_t size, const char* file, uint32_t line);
void ddr_free(void* ptr);
void* ddr_realloc(void* ptr, size_t new_size);

// 缓冲区池
data_buffer_t* ddr_buffer_acquire(const char* pool_name);
void ddr_buffer_release(data_buffer_t* buf);

// 引用计数
void ddr_ref(void* ptr);
void ddr_unref(void* ptr);

// DMA
uint64_t ddr_virt_to_phys(void* virt);
void* ddr_phys_to_virt(uint64_t phys);
int ddr_pin(void* ptr);      // 防止换出
void ddr_unpin(void* ptr);

// 统计
void ddr_get_stats(ddr_stats_t* stats);
void ddr_dump_usage(void);
```

---

## 4. 元数据管理 (MetadataManager)

### 4.1 设计目标
- 高效管理SSD元数据：L2P表、超级块信息、坏块表等
- 日志结构更新，支持崩溃恢复
- 压缩存储，减少内存占用
- 缓存管理，加速访问

### 4.2 数据结构

```c
// 元数据类型
typedef enum {
    META_TYPE_L2P,              // L2P映射表
    META_TYPE_SUPERBLOCK,       // 超级块信息
    META_TYPE_BADBLOCK,         // 坏块表
    META_TYPE_WEARLEVEL,        // 磨损信息
    META_TYPE_GC,               // GC状态
    META_TYPE_CHECKPOINT,       // 检查点
    META_TYPE_NUM
} meta_type_t;

// L2P映射条目 (8字节)
typedef struct l2p_entry {
    uint64_t ppa : 48;          // 物理页地址
    uint64_t status : 8;        // 状态位
    uint64_t reserved : 8;      // 保留
} l2p_entry_t;

#define L2P_STATUS_VALID    0x01
#define L2P_STATUS_INVALID  0x02
#define L2P_STATUS_WRITING  0x04

// 超级块元数据
typedef struct superblock_meta {
    uint32_t sb_id;             // 超级块ID
    uint32_t erase_count;       // 擦除次数
    uint32_t valid_pages;       // 有效页数
    uint32_t invalid_pages;     // 无效页数
    uint32_t free_pages;        // 空闲页数
    uint32_t state;             // 状态
    uint64_t create_time;       // 创建时间
} superblock_meta_t;

#define SB_STATE_FREE       0
#define SB_STATE_OPEN       1
#define SB_STATE_CLOSED     2
#define SB_STATE_GC_VICTIM  3

// 元数据页头
typedef struct meta_page_header {
    uint64_t sequence;          // 序列号 (用于排序)
    uint32_t type;              // 元数据类型
    uint32_t checksum;          // CRC32校验
    uint32_t data_len;          // 数据长度
    uint32_t reserved;
} meta_page_header_t;

// 元数据日志条目
typedef struct meta_log_entry {
    uint64_t sequence;          // 全局序列号
    meta_type_t type;           // 类型
    uint32_t key;               // 键 (如LBA)
    uint32_t len;               // 值长度
    uint8_t data[];             // 值数据
} meta_log_entry_t;

// 元数据缓存项
typedef struct meta_cache_entry {
    uint32_t key;               // 键
    void* data;                 // 数据指针
    uint32_t size;              // 数据大小
    uint64_t last_access;       // 最后访问时间
    uint32_t access_count;      // 访问计数
    uint8_t dirty;              // 脏标志
    uint8_t pinned;             // 固定标志
    
    // LRU链表
    struct meta_cache_entry* lru_next;
    struct meta_cache_entry* lru_prev;
    
    // Hash链表
    struct meta_cache_entry* hash_next;
} meta_cache_entry_t;

// 元数据缓存
typedef struct meta_cache {
    meta_type_t type;           // 元数据类型
    
    // Hash表 (快速查找)
    meta_cache_entry_t** hash_table;
    uint32_t hash_size;
    
    // LRU链表
    meta_cache_entry_t* lru_head;
    meta_cache_entry_t* lru_tail;
    
    // 缓存统计
    uint32_t entry_count;
    uint32_t max_entries;
    uint64_t hits;
    uint64_t misses;
    
    pthread_rwlock_t lock;
} meta_cache_t;

// 日志结构存储
typedef struct meta_log {
    uint64_t current_sequence;  // 当前序列号
    uint64_t sync_sequence;     // 已同步序列号
    
    // 日志缓冲区
    void* log_buffer;
    uint32_t buffer_used;
    uint32_t buffer_size;
    
    // 刷盘控制
    pthread_mutex_t sync_mutex;
    pthread_cond_t sync_cond;
    volatile int need_sync;
} meta_log_t;

// 检查点
typedef struct meta_checkpoint {
    uint64_t sequence;          // 检查点序列号
    uint64_t timestamp;         // 时间戳
    uint64_t write_pointer;     // 写入位置
    
    // 各类型元数据的摘要
    uint32_t l2p_count;
    uint32_t sb_count;
    uint32_t bb_count;
    
    uint32_t checksum;
} meta_checkpoint_t;

// 元数据管理器
typedef struct metadata_manager {
    // 各类元数据缓存
    meta_cache_t l2p_cache;     // L2P表缓存
    meta_cache_t sb_cache;      // 超级块缓存
    meta_cache_t bb_cache;      // 坏块表缓存
    
    // 日志
    meta_log_t meta_log;
    
    // 检查点
    meta_checkpoint_t last_checkpoint;
    uint64_t checkpoint_interval;
    
    // 后台线程
    pthread_t flush_thread;
    pthread_t checkpoint_thread;
    volatile int running;
    
    // 恢复相关
    int recovery_mode;
    uint64_t recovery_start_seq;
} metadata_manager_t;
```

### 4.3 核心算法

#### 4.3.1 L2P缓存管理
```c
// L2P查找 (带缓存)
int l2p_lookup(uint64_t lba, l2p_entry_t* entry) {
    metadata_manager_t* mgr = get_metadata_manager();
    
    // 1. 检查缓存
    meta_cache_entry_t* cache = meta_cache_get(&mgr->l2p_cache, lba);
    if (cache) {
        memcpy(entry, cache->data, sizeof(l2p_entry_t));
        mgr->l2p_cache.hits++;
        return 0;
    }
    
    mgr->l2p_cache.misses++;
    
    // 2. 从NAND读取映射页
    uint64_t map_page_ppa = get_map_page_ppa(lba);
    void* page_buffer = ddr_alloc(NAND_PAGE_SIZE, NAND_PAGE_SIZE);
    
    int ret = nand_read_page(map_page_ppa, page_buffer);
    if (ret != 0) {
        ddr_free(page_buffer);
        return -EIO;
    }
    
    // 3. 解析映射页，找到对应条目
    l2p_entry_t* entries = (l2p_entry_t*)page_buffer;
    uint32_t entry_per_page = NAND_PAGE_SIZE / sizeof(l2p_entry_t);
    uint32_t idx = lba % entry_per_page;
    
    memcpy(entry, &entries[idx], sizeof(l2p_entry_t));
    
    // 4. 加入缓存
    meta_cache_put(&mgr->l2p_cache, lba, entry, sizeof(l2p_entry_t));
    
    ddr_free(page_buffer);
    return 0;
}

// L2P更新 (日志结构)
int l2p_update(uint64_t lba, l2p_entry_t* new_entry) {
    metadata_manager_t* mgr = get_metadata_manager();
    
    // 1. 更新缓存
    meta_cache_entry_t* cache = meta_cache_get(&mgr->l2p_cache, lba);
    if (cache) {
        memcpy(cache->data, new_entry, sizeof(l2p_entry_t));
        cache->dirty = 1;
    } else {
        meta_cache_put(&mgr->l2p_cache, lba, new_entry, sizeof(l2p_entry_t));
    }
    
    // 2. 写入日志 (异步)
    meta_log_entry_t log;
    log.sequence = atomic_fetch_add(&mgr->meta_log.current_sequence, 1);
    log.type = META_TYPE_L2P;
    log.key = lba;
    log.len = sizeof(l2p_entry_t);
    
    meta_log_append(&mgr->meta_log, &log, new_entry);
    
    return 0;
}
```

#### 4.3.2 缓存LRU替换
```c
void meta_cache_put(meta_cache_t* cache, uint32_t key, void* data, uint32_t size) {
    pthread_rwlock_wrlock(&cache->lock);
    
    // 检查是否已存在
    meta_cache_entry_t* entry = hash_lookup(cache, key);
    if (entry) {
        // 更新已有条目
        memcpy(entry->data, data, size);
        entry->dirty = 1;
        lru_move_to_head(cache, entry);
        pthread_rwlock_unlock(&cache->lock);
        return;
    }
    
    // 缓存满，淘汰最久未使用
    if (cache->entry_count >= cache->max_entries) {
        meta_cache_entry_t* victim = cache->lru_tail;
        if (victim && !victim->pinned) {
            // 如果脏，先刷盘
            if (victim->dirty) {
                meta_flush_entry(cache->type, victim);
            }
            
            // 从Hash表和LRU链表移除
            hash_remove(cache, victim);
            lru_remove(cache, victim);
            
            ddr_free(victim->data);
            ddr_free(victim);
            cache->entry_count--;
        }
    }
    
    // 创建新条目
    entry = ddr_alloc(sizeof(meta_cache_entry_t), 8);
    entry->key = key;
    entry->data = ddr_alloc(size, 8);
    memcpy(entry->data, data, size);
    entry->size = size;
    entry->last_access = get_time_ns();
    entry->access_count = 1;
    entry->dirty = 1;
    entry->pinned = 0;
    
    // 加入Hash表和LRU链表
    hash_insert(cache, entry);
    lru_add_to_head(cache, entry);
    
    cache->entry_count++;
    pthread_rwlock_unlock(&cache->lock);
}
```

#### 4.3.3 检查点与恢复
```c
// 创建检查点
int meta_create_checkpoint(void) {
    metadata_manager_t* mgr = get_metadata_manager();
    
    // 1. 刷脏缓存
    meta_cache_flush_all(&mgr->l2p_cache);
    meta_cache_flush_all(&mgr->sb_cache);
    
    // 2. 等待日志同步
    meta_log_sync(&mgr->meta_log);
    
    // 3. 创建检查点
    meta_checkpoint_t cp;
    cp.sequence = mgr->meta_log.sync_sequence;
    cp.timestamp = get_time_ns();
    cp.write_pointer = get_nand_write_pointer();
    cp.l2p_count = mgr->l2p_cache.entry_count;
    cp.sb_count = mgr->sb_cache.entry_count;
    cp.checksum = crc32(&cp, sizeof(cp) - sizeof(cp.checksum));
    
    // 4. 写入检查点区域
    nand_write_checkpoint(&cp);
    
    mgr->last_checkpoint = cp;
    
    return 0;
}

// 恢复流程
int meta_recover(void) {
    metadata_manager_t* mgr = get_metadata_manager();
    
    // 1. 读取最新检查点
    meta_checkpoint_t cp;
    if (nand_read_checkpoint(&cp) != 0) {
        // 无有效检查点，全新启动
        return meta_init_fresh();
    }
    
    // 2. 验证检查点
    if (crc32(&cp, sizeof(cp) - sizeof(cp.checksum)) != cp.checksum) {
        return -EINVAL;
    }
    
    // 3. 从检查点加载基础元数据
    meta_load_from_checkpoint(&cp);
    
    // 4. 重放检查点后的日志
    uint64_t start_seq = cp.sequence;
    meta_log_replay(start_seq, mgr->meta_log.current_sequence);
    
    return 0;
}
```

### 4.4 API接口
```c
// 初始化
int meta_manager_init(void);
void meta_manager_destroy(void);

// L2P操作
int l2p_lookup(uint64_t lba, l2p_entry_t* entry);
int l2p_update(uint64_t lba, l2p_entry_t* entry);
int l2p_invalidate(uint64_t lba);
int l2p_flush(void);

// 超级块操作
int sb_get_info(uint32_t sb_id, superblock_meta_t* info);
int sb_update_info(uint32_t sb_id, superblock_meta_t* info);
int sb_alloc(uint32_t* sb_id);
int sb_free(uint32_t sb_id);

// 检查点与恢复
int meta_checkpoint(void);
int meta_recover(void);

// 统计
void meta_get_stats(meta_stats_t* stats);
```

---

## 5. 模块间交互图

```
┌─────────────────────────────────────────────────────────────────────┐
│                         TaskScheduler                                │
│                      (任务调度中心)                                   │
└──────────────────┬────────────────────────────────┬─────────────────┘
                   │                                │
        ┌──────────┴──────────┐          ┌─────────┴──────────┐
        ▼                     ▼          ▼                    ▼
┌───────────────┐     ┌───────────────┐  ┌───────────────┐  ┌───────────────┐
│  DDRManager   │◄───►│ MetadataManager│  │InterCoreComm │  │ NANDController│
│   (内存管理)   │     │   (元数据管理)  │  │  (核间通信)   │  │  (NAND控制)   │
└───────┬───────┘     └───────────────┘  └───────┬───────┘  └───────────────┘
        │                                        │
        │                                        ▼
        │                              ┌───────────────────┐
        │                              │   Other Cores     │
        │                              └───────────────────┘
        ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         Buffer Pool                                  │
│                      (数据/元数据缓冲区)                               │
└─────────────────────────────────────────────────────────────────────┘
```

---

**文档版本**: 1.0  
**创建日期**: 2026-02-06  
**状态**: 详细设计完成
