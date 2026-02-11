# SSD 模拟器 - 专有模块设计文档

## 目录
1. [L2P地址映射模块 (L2PManager)](#1-l2p地址映射模块-l2pmanager)
2. [NAND行为模拟器 (NANDSimulator)](#2-nand行为模拟器-nandsimulator)
3. [PCIe控制器 (PCIeController)](#3-pcie控制器-pciecontroller)
4. [NVMe协议模块 (NVMeModule)](#4-nvme协议模块-nvmemodule)

---

## 1. L2P地址映射模块 (L2PManager)

### 1.1 设计目标
- 高效的逻辑地址到物理地址转换
- 支持大规模映射表（TB级SSD）
- 映射表缓存管理，减少NAND访问
- 支持增量更新和持久化

### 1.2 地址空间定义

```
逻辑地址空间 (LBA)
├─ Namespace 0: LBA [0, N-1]
├─ Namespace 1: LBA [N, 2N-1]
└─ ...

物理地址空间 (PPA - Physical Page Address)
├─ Channel ID (3 bits)  : [0-7]
├─ Die ID (2 bits)      : [0-3]
├─ Plane ID (1 bit)     : [0-1]
├─ Block ID (10 bits)   : [0-1023]
├─ Page ID (9 bits)     : [0-383]
└─ Sector ID (3 bits)   : [0-7]  (16KB page / 2KB sector)

64-bit PPA格式:
[63:56] Channel ID
[55:52] Die ID
[51:48] Plane ID
[47:38] Block ID
[37:29] Page ID
[28:26] Sector ID
[25:0]  Reserved
```

### 1.3 数据结构

```c
// L2P映射条目 (压缩至8字节)
typedef struct l2p_entry {
    uint64_t ppa : 48;          // 物理页地址
    uint64_t flags : 16;        // 状态标志
} __attribute__((packed)) l2p_entry_t;

// L2P标志位
#define L2P_FLAG_VALID      (1 << 0)   // 有效
#define L2P_FLAG_INVALID    (1 << 1)   // 已失效
#define L2P_FLAG_WRITING    (1 << 2)   // 写入中
#define L2P_FLAG_GC         (1 << 3)   // GC迁移中
#define L2P_FLAG_COMPRESSED (1 << 4)   // 压缩数据

// 映射页头 (存储在NAND中)
typedef struct mapping_page_header {
    uint64_t sequence;          // 序列号
    uint32_t start_lba;         // 起始LBA
    uint32_t num_entries;       // 条目数
    uint32_t checksum;          // CRC32
    uint32_t flags;
} mapping_page_header_t;

// 映射页 (NAND_PAGE_SIZE = 16KB)
#define L2P_ENTRIES_PER_PAGE    (16384 / sizeof(l2p_entry_t))  // ~2048

// 映射缓存块 (Cache Line友好)
typedef struct l2p_cache_line {
    uint64_t start_lba;         // 起始LBA
    l2p_entry_t entries[64];    // 64个连续LBA的映射
    uint64_t last_access;
    uint32_t access_count;
    uint8_t dirty;
    uint8_t valid;
} l2p_cache_line_t;

// 映射缓存
typedef struct l2p_cache {
    l2p_cache_line_t* lines;    // 缓存行数组
    uint32_t num_lines;         // 总行数
    
    // Hash索引 (快速定位)
    uint32_t* hash_table;
    uint32_t hash_size;
    
    // LRU
    uint32_t* lru_list;
    uint32_t lru_head;
    uint32_t lru_tail;
    
    pthread_rwlock_t lock;
    
    // 统计
    uint64_t hits;
    uint64_t misses;
    uint64_t dirty_lines;
} l2p_cache_t;

// 映射表分区 (用于并行访问)
typedef struct l2p_partition {
    uint32_t start_lba;
    uint32_t end_lba;
    pthread_spinlock_t lock;
} l2p_partition_t;

// 映射日志条目 (用于增量更新)
typedef struct l2p_log_entry {
    uint64_t sequence;
    uint64_t lba;
    l2p_entry_t old_entry;
    l2p_entry_t new_entry;
} l2p_log_entry_t;

// L2P管理器
typedef struct l2p_manager {
    // 容量信息
    uint64_t num_lbas;          // 总LBA数
    uint64_t num_ppas;          // 总PPA数
    
    // 缓存
    l2p_cache_t cache;
    
    // 分区
    l2p_partition_t* partitions;
    uint32_t num_partitions;
    
    // 映射日志
    l2p_log_entry_t* log_buffer;
    uint32_t log_head;
    uint32_t log_tail;
    uint32_t log_size;
    pthread_spinlock_t log_lock;
    
    // 映射页位置表 (记录每个映射页存储的PPA)
    uint64_t* gtd;              // Global Translation Directory
    uint32_t gtd_size;
    
    // 脏映射页追踪
    uint8_t* dirty_map_pages;
    pthread_mutex_t flush_mutex;
    
    // 统计
    uint64_t total_mappings;
    uint64_t gc_migrations;
} l2p_manager_t;
```

### 1.4 核心算法

#### 1.4.1 L2P查找
```c
int l2p_lookup(l2p_manager_t* l2p, uint64_t lba, l2p_entry_t* entry) {
    // 1. 检查缓存
    uint32_t line_idx = l2p_cache_lookup(&l2p->cache, lba);
    if (line_idx != UINT32_MAX) {
        uint32_t offset = lba % 64;
        *entry = l2p->cache.lines[line_idx].entries[offset];
        
        // 更新访问统计
        l2p->cache.lines[line_idx].last_access = get_time_ns();
        l2p->cache.lines[line_idx].access_count++;
        l2p_cache_update_lru(&l2p->cache, line_idx);
        
        l2p->cache.hits++;
        return (entry->flags & L2P_FLAG_VALID) ? 0 : -ENOENT;
    }
    
    l2p->cache.misses++;
    
    // 2. 从NAND加载映射页
    uint32_t map_page_id = lba / L2P_ENTRIES_PER_PAGE;
    
    // 分配缓存行
    line_idx = l2p_cache_alloc(&l2p->cache);
    l2p_cache_line_t* line = &l2p->cache.lines[line_idx];
    
    // 加载映射页数据
    uint64_t map_ppa = l2p->gtd[map_page_id];
    if (map_ppa == INVALID_PPA) {
        // 映射页不存在，返回无效
        memset(entry, 0, sizeof(l2p_entry_t));
        return -ENOENT;
    }
    
    // 读取NAND
    void* page_buffer = ddr_alloc(NAND_PAGE_SIZE, NAND_PAGE_SIZE);
    int ret = nand_read_page(map_ppa, page_buffer);
    if (ret != 0) {
        ddr_free(page_buffer);
        return ret;
    }
    
    // 解析映射页
    mapping_page_header_t* header = (mapping_page_header_t*)page_buffer;
    l2p_entry_t* entries = (l2p_entry_t*)((uint8_t*)page_buffer + sizeof(mapping_page_header_t));
    
    // 填充缓存行 (64个连续LBA)
    uint64_t start_lba = (lba / 64) * 64;
    line->start_lba = start_lba;
    line->valid = 1;
    line->dirty = 0;
    line->last_access = get_time_ns();
    line->access_count = 1;
    
    for (uint32_t i = 0; i < 64; i++) {
        uint32_t entry_idx = start_lba + i - header->start_lba;
        if (entry_idx < header->num_entries) {
            line->entries[i] = entries[entry_idx];
        } else {
            memset(&line->entries[i], 0, sizeof(l2p_entry_t));
        }
    }
    
    // 返回结果
    uint32_t offset = lba % 64;
    *entry = line->entries[offset];
    
    ddr_free(page_buffer);
    
    return (entry->flags & L2P_FLAG_VALID) ? 0 : -ENOENT;
}
```

#### 1.4.2 L2P更新
```c
int l2p_update(l2p_manager_t* l2p, uint64_t lba, uint64_t new_ppa, uint16_t flags) {
    uint32_t partition_id = lba % l2p->num_partitions;
    l2p_partition_t* part = &l2p->partitions[partition_id];
    
    pthread_spin_lock(&part->lock);
    
    // 1. 获取旧映射
    l2p_entry_t old_entry;
    int ret = l2p_lookup(l2p, lba, &old_entry);
    
    // 2. 更新缓存
    uint32_t line_idx = l2p_cache_lookup(&l2p->cache, lba);
    if (line_idx == UINT32_MAX) {
        // 缓存未命中，加载后再更新
        line_idx = l2p_cache_load(l2p, lba);
    }
    
    l2p_cache_line_t* line = &l2p->cache.lines[line_idx];
    uint32_t offset = lba % 64;
    
    // 记录旧值到日志
    l2p_log_entry_t log = {
        .sequence = get_next_sequence(),
        .lba = lba,
        .old_entry = line->entries[offset],
        .new_entry = { .ppa = new_ppa, .flags = flags | L2P_FLAG_VALID }
    };
    l2p_log_append(l2p, &log);
    
    // 更新条目
    line->entries[offset].ppa = new_ppa;
    line->entries[offset].flags = flags | L2P_FLAG_VALID;
    line->dirty = 1;
    line->last_access = get_time_ns();
    
    // 标记映射页为脏
    uint32_t map_page_id = lba / L2P_ENTRIES_PER_PAGE;
    l2p->dirty_map_pages[map_page_id / 8] |= (1 << (map_page_id % 8));
    
    pthread_spin_unlock(&part->lock);
    
    // 3. 如果旧映射有效，标记为无效并通知GC
    if (ret == 0 && (old_entry.flags & L2P_FLAG_VALID)) {
        // 标记旧物理页为无效
        uint64_t old_ppa = old_entry.ppa;
        uint32_t old_channel = PPA_GET_CHANNEL(old_ppa);
        uint32_t old_die = PPA_GET_DIE(old_ppa);
        uint32_t old_block = PPA_GET_BLOCK(old_ppa);
        uint32_t old_page = PPA_GET_PAGE(old_ppa);
        
        // 更新超级块的无效页计数
        superblock_invalidate_page(old_block, old_page);
    }
    
    return 0;
}
```

#### 1.4.3 映射表刷盘
```c
int l2p_flush(l2p_manager_t* l2p) {
    pthread_mutex_lock(&l2p->flush_mutex);
    
    // 1. 收集所有脏缓存行
    uint32_t* dirty_lines = malloc(l2p->cache.num_lines * sizeof(uint32_t));
    uint32_t num_dirty = 0;
    
    for (uint32_t i = 0; i < l2p->cache.num_lines; i++) {
        if (l2p->cache.lines[i].valid && l2p->cache.lines[i].dirty) {
            dirty_lines[num_dirty++] = i;
        }
    }
    
    // 2. 按映射页ID分组
    uint32_t* map_page_lines[L2P_MAX_MAP_PAGES] = {0};
    uint32_t map_page_counts[L2P_MAX_MAP_PAGES] = {0};
    
    for (uint32_t i = 0; i < num_dirty; i++) {
        l2p_cache_line_t* line = &l2p->cache.lines[dirty_lines[i]];
        uint32_t map_page_id = line->start_lba / L2P_ENTRIES_PER_PAGE;
        
        // 简单起见，这里直接写入NAND
        // 实际实现需要合并同一映射页的多个缓存行
    }
    
    // 3. 写入NAND
    for (uint32_t i = 0; i < num_dirty; i++) {
        l2p_cache_line_t* line = &l2p->cache.lines[dirty_lines[i]];
        
        // 分配新的映射页
        uint64_t new_map_ppa;
        int ret = ppa_alloc(&new_map_ppa, 1, PAGE_TYPE_MAPPING);
        if (ret != 0) {
            break;
        }
        
        // 构建映射页
        void* page_buffer = ddr_alloc(NAND_PAGE_SIZE, NAND_PAGE_SIZE);
        mapping_page_header_t* header = (mapping_page_header_t*)page_buffer;
        header->sequence = get_next_sequence();
        header->start_lba = line->start_lba;
        header->num_entries = 64;
        header->checksum = 0;
        header->flags = 0;
        
        memcpy((uint8_t*)page_buffer + sizeof(mapping_page_header_t),
               line->entries, 64 * sizeof(l2p_entry_t));
        
        header->checksum = crc32(page_buffer, NAND_PAGE_SIZE);
        
        // 写入NAND
        nand_write_page(new_map_ppa, page_buffer);
        
        // 更新GTD
        uint32_t map_page_id = line->start_lba / L2P_ENTRIES_PER_PAGE;
        l2p->gtd[map_page_id] = new_map_ppa;
        
        // 标记缓存行干净
        line->dirty = 0;
        l2p->cache.dirty_lines--;
        
        ddr_free(page_buffer);
    }
    
    // 4. 更新检查点
    meta_checkpoint();
    
    free(dirty_lines);
    pthread_mutex_unlock(&l2p->flush_mutex);
    
    return 0;
}
```

### 1.5 API接口
```c
// 初始化
int l2p_manager_init(uint64_t num_lbas, uint32_t cache_lines);
void l2p_manager_destroy(void);

// 核心操作
int l2p_lookup(uint64_t lba, l2p_entry_t* entry);
int l2p_update(uint64_t lba, uint64_t ppa, uint16_t flags);
int l2p_invalidate(uint64_t lba);
int l2p_flush(void);

// 批量操作
int l2p_lookup_batch(uint64_t* lbas, l2p_entry_t* entries, uint32_t count);
int l2p_update_batch(uint64_t* lbas, uint64_t* ppas, uint16_t* flags, uint32_t count);

// GC支持
int l2p_get_victim_blocks(uint32_t** victim_blocks, uint32_t* count);
int l2p_migrate(uint64_t lba, uint64_t new_ppa);

// 统计
void l2p_get_stats(l2p_stats_t* stats);
```

---

## 2. NAND行为模拟器 (NANDSimulator)

### 2.1 设计目标
- 精确模拟NAND Flash时序特性
- 支持多种NAND类型 (SLC/MLC/TLC/QLC)
- 模拟并行性 (Multi-Channel, Multi-Die)

- 支持错误注入和可靠性模拟
- 可配置的延迟模型

### 2.2 NAND组织结构

```
NAND Device
├── Channel 0
│   ├── Die 0
│   │   ├── Plane 0 (2 Planes per Die)
│   │   │   ├── Block 0 (1024 Blocks per Plane)
│   │   │   │   ├── Page 0 (384 Pages per Block)
│   │   │   │   │   ├── Data Area (16KB)
│   │   │   │   │   └── OOB Area (1664B) - ECC/Metadata
│   │   │   │   ├── Page 1
│   │   │   │   └── ...
│   │   │   └── Block 1
│   │   └── Plane 1
│   └── Die 1
├── Channel 1
│   └── ...
└── Channel 7

总容量计算:
8 Channels × 4 Dies × 2 Planes × 1024 Blocks × 384 Pages × 16KB = 384GB
```

### 2.3 数据结构

```c
// NAND时序参数 (微秒)
typedef struct nand_timing {
    uint32_t t_read;            // 页读取时间
    uint32_t t_prog;            // 页编程时间
    uint32_t t_erase;           // 块擦除时间
    uint32_t t_transfer;        // 数据传输时间 (16KB)
    
    // 高级时序
    uint32_t t_r_multiplane;    // Multi-plane读优化
    uint32_t t_p_multiplane;    // Multi-plane编程优化
    uint32_t t_e_multiplane;    // Multi-plane擦除优化
    uint32_t t_ccs;             // Change Column Setup
    uint32_t t_adl;             // Address to Data Loading
} nand_timing_t;

// NAND类型配置
typedef enum {
    NAND_TYPE_SLC,
    NAND_TYPE_MLC,
    NAND_TYPE_TLC,
    NAND_TYPE_QLC,
    NAND_TYPE_NUM
} nand_type_t;

typedef struct nand_type_config {
    nand_type_t type;
    const char* name;
    uint32_t bits_per_cell;
    uint32_t pages_per_block;
    uint32_t page_size;
    uint32_t oob_size;
    nand_timing_t timing;
    uint32_t pe_cycles;         // 擦写寿命
    double ber;                 // Bit Error Rate
} nand_type_config_t;

// 页状态
typedef enum {
    PAGE_STATE_FREE,            // 空闲
    PAGE_STATE_VALID,           // 有效数据
    PAGE_STATE_INVALID,         // 无效数据
    PAGE_STATE_WRITING,         // 写入中
} page_state_t;

// 块状态
typedef enum {
    BLOCK_STATE_FREE,           // 空闲
    BLOCK_STATE_OPEN,           // 开放中 (正在写入)
    BLOCK_STATE_FULL,           // 已满
    BLOCK_STATE_GC,             // GC中
    BLOCK_STATE_BAD,            // 坏块
} block_state_t;

// NAND页模拟
typedef struct nand_page {
    uint8_t* data;              // 数据区指针
    uint8_t* oob;               // OOB区指针
    page_state_t state;         // 页状态
    uint64_t written_time;      // 写入时间戳
    uint32_t write_count;       // 写入次数 (重写计数)
} nand_page_t;

// NAND块模拟
typedef struct nand_block {
    nand_page_t* pages;         // 页数组
    uint32_t num_pages;         // 页数
    block_state_t state;        // 块状态
    uint32_t erase_count;       // 擦除次数
    uint32_t valid_pages;       // 有效页数
    uint32_t invalid_pages;     // 无效页数
    uint32_t free_pages;        // 空闲页数
    uint64_t last_erase_time;   // 最后擦除时间
} nand_block_t;

// NAND Plane模拟
typedef struct nand_plane {
    nand_block_t* blocks;       // 块数组
    uint32_t num_blocks;        // 块数
    uint32_t open_block;        // 当前开放块
    uint32_t next_page;         // 下一写入页
    
    // Plane级状态
    uint8_t busy;               // 忙标志
    uint64_t busy_until;        // 忙到何时
} nand_plane_t;

// NAND Die模拟
typedef struct nand_die {
    nand_plane_t* planes;       // Plane数组
    uint32_t num_planes;        // Plane数
    uint32_t active_planes;     // 活跃Plane数 (Multi-plane操作)
    
    // Die级命令队列
    nand_cmd_t* cmd_queue;
    uint32_t queue_head;
    uint32_t queue_tail;
    pthread_mutex_t queue_lock;
} nand_die_t;

// NAND Channel模拟
typedef struct nand_channel {
    nand_die_t* dies;           // Die数组
    uint32_t num_dies;          // Die数
    uint32_t active_die;        // 当前选中Die
    
    // Channel级状态
    uint8_t busy;
    uint64_t busy_until;
    uint32_t bandwidth;         // 带宽 (MB/s)
} nand_channel_t;

// NAND命令
typedef enum {
    NAND_CMD_READ,              // 页读取
    NAND_CMD_READ_MULTIPLANE,   // Multi-plane读取
    NAND_CMD_PROG,              // 页编程
    NAND_CMD_PROG_MULTIPLANE,   // Multi-plane编程
    NAND_CMD_ERASE,             // 块擦除
    NAND_CMD_ERASE_MULTIPLANE,  // Multi-plane擦除
    NAND_CMD_READ_ID,           // 读ID
    NAND_CMD_READ_STATUS,       // 读状态
    NAND_CMD_RESET,             // 复位
} nand_cmd_type_t;

typedef struct nand_cmd {
    nand_cmd_type_t type;
    uint64_t ppa;               // 物理页地址
    void* data_buf;             // 数据缓冲区
    void* oob_buf;              // OOB缓冲区
    uint64_t submit_time;       // 提交时间
    uint64_t complete_time;     // 完成时间
    void (*callback)(struct nand_cmd*);
    void* private_data;
} nand_cmd_t;

// NAND模拟器
typedef struct nand_simulator {
    nand_channel_t* channels;   // Channel数组
    uint32_t num_channels;      // Channel数
    
    // 配置
    nand_type_config_t* type_config;
    uint32_t dies_per_channel;
    uint32_t planes_per_die;
    uint32_t blocks_per_plane;
    
    // 模拟线程
    pthread_t sim_thread;
    volatile int running;
    
    // 时间推进
    uint64_t current_time;      // 当前模拟时间 (纳秒)
    pthread_mutex_t time_lock;
    
    // 错误注入
    double error_rate;          // 错误率
    uint32_t inject_read_error; // 读错误注入
    uint32_t inject_prog_error; // 编程错误注入
    uint32_t inject_erase_error;// 擦除错误注入
    
    // 统计
    uint64_t read_count;
    uint64_t prog_count;
    uint64_t erase_count;
    uint64_t read_error_count;
    uint64_t prog_error_count;
    uint64_t erase_error_count;
    uint64_t total_read_latency;
    uint64_t total_prog_latency;
    uint64_t total_erase_latency;
} nand_simulator_t;
```

### 2.4 核心算法

#### 2.4.1 NAND初始化
```c
int nand_sim_init(nand_simulator_t* sim, nand_config_t* config) {
    sim->num_channels = config->num_channels;
    sim->dies_per_channel = config->dies_per_channel;
    sim->planes_per_die = config->planes_per_die;
    sim->blocks_per_plane = config->blocks_per_plane;
    
    // 加载NAND类型配置
    sim->type_config = &g_nand_configs[config->type];
    
    // 分配Channel数组
    sim->channels = calloc(sim->num_channels, sizeof(nand_channel_t));
    
    for (uint32_t ch = 0; ch < sim->num_channels; ch++) {
        nand_channel_t* channel = &sim->channels[ch];
        channel->num_dies = sim->dies_per_channel;
        channel->dies = calloc(channel->num_dies, sizeof(nand_die_t));
        channel->bandwidth = config->channel_bandwidth;
        
        for (uint32_t die = 0; die < channel->num_dies; die++) {
            nand_die_t* d = &channel->dies[die];
            d->num_planes = sim->planes_per_die;
            d->planes = calloc(d->num_planes, sizeof(nand_plane_t));
            
            for (uint32_t pl = 0; pl < d->num_planes; pl++) {
                nand_plane_t* plane = &d->planes[pl];
                plane->num_blocks = sim->blocks_per_plane;
                plane->blocks = calloc(plane->num_blocks, sizeof(nand_block_t));
                plane->open_block = UINT32_MAX;
                
                for (uint32_t blk = 0; blk < plane->num_blocks; blk++) {
                    nand_block_t* block = &plane->blocks[blk];
                    block->num_pages = sim->type_config->pages_per_block;
                    block->pages = calloc(block->num_pages, sizeof(nand_page_t));
                    block->state = BLOCK_STATE_FREE;
                    
                    for (uint32_t pg = 0; pg < block->num_pages; pg++) {
                        nand_page_t* page = &block->pages[pg];
                        page->data = calloc(1, sim->type_config->page_size);
                        page->oob = calloc(1, sim->type_config->oob_size);
                        page->state = PAGE_STATE_FREE;
                    }
                }
            }
        }
    }
    
    // 启动模拟线程
    sim->running = 1;
    pthread_create(&sim->sim_thread, NULL, nand_sim_thread, sim);
    
    return 0;
}
```

#### 2.4.2 NAND读取模拟
```c
int nand_sim_read(nand_simulator_t* sim, uint64_t ppa, void* data_buf, void* oob_buf) {
    // 解析PPA
    uint32_t ch = PPA_GET_CHANNEL(ppa);
    uint32_t die = PPA_GET_DIE(ppa);
    uint32_t pl = PPA_GET_PLANE(ppa);
    uint32_t blk = PPA_GET_BLOCK(ppa);
    uint32_t pg = PPA_GET_PAGE(ppa);
    
    if (ch >= sim->num_channels) return -EINVAL;
    
    nand_channel_t* channel = &sim->channels[ch];
    nand_die_t* d = &channel->dies[die];
    nand_plane_t* plane = &d->planes[pl];
    nand_block_t* block = &plane->blocks[blk];
    nand_page_t* page = &block->pages[pg];
    
    // 检查状态
    if (block->state == BLOCK_STATE_BAD) {
        return -EIO;
    }
    
    if (page->state != PAGE_STATE_VALID && page->state != PAGE_STATE_INVALID) {
        // 读取空页或未写入页
        memset(data_buf, 0xFF, sim->type_config->page_size);
        memset(oob_buf, 0xFF, sim->type_config->oob_size);
        return 0;
    }
    
    // 计算延迟
    uint64_t latency_ns = sim->type_config->timing.t_read * 1000;
    
    // 数据传输时间
    uint32_t transfer_time_us = (sim->type_config->page_size + sim->type_config->oob_size) 
                                * 1000000 / (channel->bandwidth * 1024 * 1024);
    latency_ns += transfer_time_us * 1000;
    
    // 模拟时间推进
    uint64_t complete_time = sim->current_time + latency_ns;
    
    // 错误注入
    if (sim->inject_read_error || (sim->error_rate > 0 && random_double() < sim->error_rate)) {
        // 注入位翻转
        inject_bit_errors(data_buf, sim->type_config->page_size, sim->error_rate);
        sim->read_error_count++;
    }
    
    // 复制数据
    memcpy(data_buf, page->data, sim->type_config->page_size);
    memcpy(oob_buf, page->oob, sim->type_config->oob_size);
    
    // 更新统计
    sim->read_count++;
    sim->total_read_latency += latency_ns;
    
    return 0;
}
```

#### 2.4.3 NAND编程模拟
```c
int nand_sim_program(nand_simulator_t* sim, uint64_t ppa, void* data_buf, void* oob_buf) {
    // 解析PPA
    uint32_t ch = PPA_GET_CHANNEL(ppa);
    uint32_t die = PPA_GET_DIE(ppa);
    uint32_t pl = PPA_GET_PLANE(ppa);
    uint32_t blk = PPA_GET_BLOCK(ppa);
    uint32_t pg = PPA_GET_PAGE(ppa);
    
    nand_channel_t* channel = &sim->channels[ch];
    nand_die_t* d = &channel->dies[die];
    nand_plane_t* plane = &d->planes[pl];
    nand_block_t* block = &plane->blocks[blk];
    nand_page_t* page = &block->pages[pg];
    
    // 检查是否可以编程
    if (block->state == BLOCK_STATE_BAD) {
        return -EIO;
    }
    
    if (page->state != PAGE_STATE_FREE) {
        // NAND不允许覆盖写
        return -EEXIST;
    }
    
    // 必须按顺序写入 (仅在open block中)
    if (block->state != BLOCK_STATE_OPEN && block->state != BLOCK_STATE_FREE) {
        return -EINVAL;
    }
    
    // 检查顺序写入约束
    if (block->state == BLOCK_STATE_OPEN && pg != plane->next_page) {
        return -EINVAL;
    }
    
    // 计算延迟
    uint64_t latency_ns = sim->type_config->timing.t_prog * 1000;
    
    // 数据传输时间
    uint32_t transfer_time_us = (sim->type_config->page_size + sim->type_config->oob_size) 
                                * 1000000 / (channel->bandwidth * 1024 * 1024);
    latency_ns += transfer_time_us * 1000;
    
    // 错误注入
    if (sim->inject_prog_error || (sim->error_rate > 0 && random_double() < sim->error_rate)) {
        sim->prog_error_count++;
        return -EIO;
    }
    
    // 写入数据
    memcpy(page->data, data_buf, sim->type_config->page_size);
    memcpy(page->oob, oob_buf, sim->type_config->oob_size);
    
    // 更新页状态
    page->state = PAGE_STATE_VALID;
    page->written_time = sim->current_time;
    page->write_count++;
    
    // 更新块状态
    block->valid_pages++;
    block->free_pages--;
    
    if (block->state == BLOCK_STATE_FREE) {
        block->state = BLOCK_STATE_OPEN;
        plane->open_block = blk;
    }
    
    plane->next_page = pg + 1;
    
    if (plane->next_page >= block->num_pages) {
        block->state = BLOCK_STATE_FULL;
        plane->open_block = UINT32_MAX;
        plane->next_page = 0;
    }
    
    // 更新统计
    sim->prog_count++;
    sim->total_prog_latency += latency_ns;
    
    return 0;
}
```

#### 2.4.4 NAND擦除模拟
```c
int nand_sim_erase(nand_simulator_t* sim, uint64_t ppa) {
    // 擦除以块为单位，PPA中的page字段被忽略
    uint32_t ch = PPA_GET_CHANNEL(ppa);
    uint32_t die = PPA_GET_DIE(ppa);
    uint32_t pl = PPA_GET_PLANE(ppa);
    uint32_t blk = PPA_GET_BLOCK(ppa);
    
    nand_channel_t* channel = &sim->channels[ch];
    nand_die_t* d = &channel->dies[die];
    nand_plane_t* plane = &d->planes[pl];
    nand_block_t* block = &plane->blocks[blk];
    
    // 计算延迟
    uint64_t latency_ns = sim->type_config->timing.t_erase * 1000;
    
    // 错误注入
    if (sim->inject_erase_error) {
        // 标记为坏块
        block->state = BLOCK_STATE_BAD;
        sim->erase_error_count++;
        return -EIO;
    }
    
    // 擦除所有页
    for (uint32_t pg = 0; pg < block->num_pages; pg++) {
        nand_page_t* page = &block->pages[pg];
        memset(page->data, 0xFF, sim->type_config->page_size);
        memset(page->oob, 0xFF, sim->type_config->oob_size);
        page->state = PAGE_STATE_FREE;
        page->write_count = 0;
    }
    
    // 更新块状态
    block->state = BLOCK_STATE_FREE;
    block->erase_count++;
    block->valid_pages = 0;
    block->invalid_pages = 0;
    block->free_pages = block->num_pages;
    block->last_erase_time = sim->current_time;
    
    // 更新统计
    sim->erase_count++;
    sim->total_erase_latency += latency_ns;
    
    return 0;
}
```

### 2.5 Multi-Plane操作优化

```c
// Multi-Plane编程 (并行编程多个plane)
int nand_sim_program_multiplane(nand_simulator_t* sim, 
                                uint64_t* ppas, void** data_bufs, void** oob_bufs,
                                uint32_t count) {
    if (count < 2 || count > MAX_PLANES) {
        return -EINVAL;
    }
    
    // 验证所有PPA在同一Die的不同Plane
    uint32_t ch = PPA_GET_CHANNEL(ppas[0]);
    uint32_t die = PPA_GET_DIE(ppas[0]);
    
    for (uint32_t i = 1; i < count; i++) {
        if (PPA_GET_CHANNEL(ppas[i]) != ch || PPA_GET_DIE(ppas[i]) != die) {
            return -EINVAL;
        }
        if (PPA_GET_PLANE(ppas[i]) == PPA_GET_PLANE(ppas[i-1])) {
            return -EINVAL;  // 必须在不同Plane
        }
    }
    
    // Multi-Plane编程延迟优化
    // 第一个页正常编程时间，后续页只需增量时间
    uint64_t base_latency = sim->type_config->timing.t_prog * 1000;
    uint64_t inc_latency = sim->type_config->timing.t_p_multiplane * 1000;
    uint64_t total_latency = base_latency + (count - 1) * inc_latency;
    
    // 执行编程
    for (uint32_t i = 0; i < count; i++) {
        nand_sim_program(sim, ppas[i], data_bufs[i], oob_bufs[i]);
    }
    
    return 0;
}
```

### 2.6 API接口
```c
// 初始化
int nand_sim_init(nand_simulator_t* sim, nand_config_t* config);
void nand_sim_destroy(nand_simulator_t* sim);

// 基本操作
int nand_read_page(uint64_t ppa, void* data_buf);
int nand_read_page_oob(uint64_t ppa, void* oob_buf);
int nand_write_page(uint64_t ppa, void* data_buf, void* oob_buf);
int nand_erase_block(uint64_t ppa);

// Multi-plane操作
int nand_read_multiplane(uint64_t* ppas, void** data_bufs, uint32_t count);
int nand_write_multiplane(uint64_t* ppas, void** data_bufs, void** oob_bufs, uint32_t count);
int nand_erase_multiplane(uint64_t* ppas, uint32_t count);

// 状态查询
int nand_get_page_status(uint64_t ppa);
int nand_get_block_status(uint64_t ppa);

// 错误注入
void nand_inject_read_error(uint64_t ppa);
void nand_inject_write_error(uint64_t ppa);
void nand_inject_erase_error(uint64_t ppa);
void nand_set_error_rate(double rate);

// 统计
void nand_get_stats(nand_stats_t* stats);
void nand_reset_stats(void);
```

---

## 3. PCIe控制器 (PCIeController)

### 3.1 设计目标
- 实现PCIe Endpoint功能
- 支持配置空间和BAR空间访问
- DMA数据传输支持
- 中断处理机制

### 3.2 PCIe配置空间

```c
// PCIe Configuration Space Header (64 bytes)
typedef struct pcie_config_space {
    // Vendor ID & Device ID
    uint16_t vendor_id;         // 0x00
    uint16_t device_id;         // 0x02
    
    // Command & Status
    uint16_t command;           // 0x04
    uint16_t status;            // 0x06
    
    // Revision & Class Code
    uint8_t  revision_id;       // 0x08
    uint8_t  prog_if;           // 0x09
    uint8_t  subclass;          // 0x0A
    uint8_t  class_code;        // 0x0B
    
    // Cache Line & Latency
    uint8_t  cache_line_size;   // 0x0C
    uint8_t  latency_timer;     // 0x0D
    uint8_t  header_type;       // 0x0E
    uint8_t  bist;              // 0x0F
    
    // BAR0 - BAR5
    uint32_t bar[6];            // 0x10 - 0x27
    
    // CardBus CIS Pointer
    uint32_t cardbus_cis;       // 0x28
    
    // Subsystem IDs
    uint16_t subsys_vendor_id;  // 0x2C
    uint16_t subsys_id;         // 0x2E
    
    // ROM BAR
    uint32_t rom_bar;           // 0x30
    
    // Capabilities
    uint8_t  capabilities_ptr;  // 0x34
    uint8_t  reserved[7];       // 0x35 - 0x3B
    
    // Interrupt
    uint8_t  interrupt_line;    // 0x3C
    uint8_t  interrupt_pin;     // 0x3D
    uint16_t min_gnt;           // 0x3E
    uint16_t max_lat;           // 0x3F
} __attribute__((packed)) pcie_config_space_t;

// PCIe Capabilities
typedef struct pcie_capability {
    uint8_t  cap_id;            // 0x10 = PCI Express Capability
    uint8_t  next_ptr;
    uint16_t pcie_cap;
    uint32_t dev_cap;
    uint16_t dev_ctrl;
    uint16_t dev_status;
    uint32_t link_cap;
    uint16_t link_ctrl;
    uint16_t link_status;
    // ... more fields
} __attribute__((packed)) pcie_capability_t;

// MSI/MSI-X Capability
typedef struct msi_capability {
    uint8_t  cap_id;            // 0x05 = MSI
    uint8_t  next_ptr;
    uint16_t msg_ctrl;
    uint32_t msg_addr_low;
    uint32_t msg_addr_high;
    uint16_t msg_data;
    // ...
} __attribute__((packed)) msi_capability_t;
```

### 3.3 数据结构

```c
// BAR Region
typedef struct pcie_bar_region {
    uint64_t base_addr;         // 基地址
    uint64_t size;              // 大小
    uint32_t type;              // 类型 (IO/Mem32/Mem64)
    uint8_t  prefetchable;      // 可预取
    
    // 访问回调
    uint32_t (*read32)(uint64_t offset);
    void (*write32)(uint64_t offset, uint32_t value);
    uint64_t (*read64)(uint64_t offset);
    void (*write64)(uint64_t offset, uint64_t value);
} pcie_bar_region_t;

// DMA描述符
typedef struct dma_descriptor {
    uint64_t src_addr;          // 源地址
    uint64_t dst_addr;          // 目标地址
    uint32_t length;            // 长度
    uint32_t flags;             // 标志
    uint64_t next_desc;         // 下一个描述符
} __attribute__((packed)) dma_descriptor_t;

// DMA引擎
typedef struct dma_engine {
    uint8_t  id;                // DMA通道ID
    uint8_t  busy;              // 忙标志
    uint64_t desc_addr;         // 描述符链表地址
    uint32_t desc_count;        // 描述符数量
    
    // 状态
    uint64_t bytes_transferred;
    uint32_t errors;
    
    // 线程
    pthread_t thread;
    volatile int running;
} dma_engine_t;

// TLP (Transaction Layer Packet) Header
typedef struct tlp_header {
    uint32_t dw0;               // Format, Type, Length
    uint32_t dw1;               // Requester ID, Tag, BE
    uint32_t dw2;               // Address (low)
    uint32_t dw3;               // Address (high) - for 64-bit
} __attribute__((packed)) tlp_header_t;

// PCIe Endpoint
typedef struct pcie_endpoint {
    // 配置空间
    pcie_config_space_t config_space;
    uint8_t* ext_config_space;  // 扩展配置空间 (4KB)
    
    // BAR区域
    pcie_bar_region_t bars[6];
    
    // DMA引擎
    dma_engine_t* dma_engines;
    uint32_t num_dma_engines;
    
    // 中断
    uint8_t int_pin;            // 传统中断引脚
    uint8_t msi_enabled;        // MSI使能
    uint8_t msix_enabled;       // MSI-X使能
    uint32_t msi_addr_low;
    uint32_t msi_addr_high;
    uint16_t msi_data;
    
    // 链路状态
    uint32_t link_speed;        // 链路速度 (GT/s)
    uint32_t link_width;        // 链路宽度 (x1, x4, x8, x16)
    uint64_t max_payload;       // Max Payload Size
    uint64_t max_read_req;      // Max Read Request Size
    
    // 统计
    uint64_t tlp_sent;
    uint64_t tlp_received;
    uint64_t dma_bytes_read;
    uint64_t dma_bytes_written;
} pcie_endpoint_t;

// PCIe控制器
typedef struct pcie_controller {
    pcie_endpoint_t endpoint;
    
    // 模拟参数
    uint64_t latency_ns;        // 传输延迟
    uint64_t bandwidth;         // 带宽 (bytes/s)
    
    // 回调函数 (与主机交互)
    void* host_mmio_base;       // 主机MMIO基地址
    int (*host_dma_read)(uint64_t host_addr, void* buf, size_t len);
    int (*host_dma_write)(uint64_t host_addr, void* buf, size_t len);
    void (*host_interrupt)(uint32_t vector);
} pcie_controller_t;
```

### 3.4 核心算法

#### 3.4.1 配置空间访问
```c
uint32_t pcie_config_read32(pcie_controller_t* pcie, uint16_t offset) {
    if (offset < sizeof(pcie_config_space_t)) {
        return *(uint32_t*)((uint8_t*)&pcie->endpoint.config_space + offset);
    } else if (offset < 4096) {
        return *(uint32_t*)(pcie->endpoint.ext_config_space + offset - sizeof(pcie_config_space_t));
    }
    return 0xFFFFFFFF;
}

void pcie_config_write32(pcie_controller_t* pcie, uint16_t offset, uint32_t value) {
    if (offset < sizeof(pcie_config_space_t)) {
        // 处理特殊寄存器
        switch (offset) {
            case 0x04:  // Command
                pcie->endpoint.config_space.command = value & 0xFFFF;
                break;
            case 0x10:  // BAR0
            case 0x14:  // BAR1
            case 0x18:  // BAR2
            case 0x1C:  // BAR3
            case 0x20:  // BAR4
            case 0x24:  // BAR5
                pcie_bar_write(pcie, (offset - 0x10) / 4, value);
                break;
            default:
                *(uint32_t*)((uint8_t*)&pcie->endpoint.config_space + offset) = value;
                break;
        }
    }
}

// BAR处理
void pcie_bar_write(pcie_controller_t* pcie, uint32_t bar_idx, uint32_t value) {
    pcie_bar_region_t* bar = &pcie->endpoint.bars[bar_idx];
    
    if (value == 0xFFFFFFFF) {
        // 大小探测
        pcie->endpoint.config_space.bar[bar_idx] = ~(bar->size - 1);
    } else {
        // 设置基地址
        bar->base_addr = value & ~(bar->size - 1);
        pcie->endpoint.config_space.bar[bar_idx] = value;
    }
}
```

#### 3.4.2 MMIO访问处理
```c
uint32_t pcie_mmio_read32(pcie_controller_t* pcie, uint64_t addr) {
    // 确定访问哪个BAR
    for (int i = 0; i < 6; i++) {
        pcie_bar_region_t* bar = &pcie->endpoint.bars[i];
        if (addr >= bar->base_addr && addr < bar->base_addr + bar->size) {
            uint64_t offset = addr - bar->base_addr;
            if (bar->read32) {
                return bar->read32(offset);
            }
            break;
        }
    }
    return 0xFFFFFFFF;
}

void pcie_mmio_write32(pcie_controller_t* pcie, uint64_t addr, uint32_t value) {
    for (int i = 0; i < 6; i++) {
        pcie_bar_region_t* bar = &pcie->endpoint.bars[i];
        if (addr >= bar->base_addr && addr < bar->base_addr + bar->size) {
            uint64_t offset = addr - bar->base_addr;
            if (bar->write32) {
                bar->write32(offset, value);
            }
            break;
        }
    }
}
```

#### 3.4.3 DMA操作
```c
// 提交DMA传输
int pcie_dma_submit(pcie_controller_t* pcie, uint32_t dma_id, 
                    uint64_t desc_addr, uint32_t desc_count) {
    dma_engine_t* dma = &pcie->endpoint.dma_engines[dma_id];
    
    if (dma->busy) {
        return -EBUSY;
    }
    
    dma->desc_addr = desc_addr;
    dma->desc_count = desc_count;
    dma->busy = 1;
    
    // 启动DMA线程处理
    pthread_cond_signal(&dma_cond);
    
    return 0;
}

// DMA执行线程
void* dma_thread(void* arg) {
    dma_engine_t* dma = (dma_engine_t*)arg;
    pcie_controller_t* pcie = container_of(dma, pcie_controller_t, endpoint.dma_engines[dma->id]);
    
    while (dma->running) {
        pthread_mutex_lock(&dma_mutex);
        while (!dma->busy && dma->running) {
            pthread_cond_wait(&dma_cond, &dma_mutex);
        }
        pthread_mutex_unlock(&dma_mutex);
        
        if (!dma->running) break;
        
        // 处理描述符链
        uint64_t desc_ptr = dma->desc_addr;
        uint32_t desc_processed = 0;
        
        while (desc_processed < dma->desc_count && desc_ptr != 0) {
            // 从主机读取描述符
            dma_descriptor_t desc;
            pcie->host_dma_read(desc_ptr, &desc, sizeof(desc));
            
            // 确定传输方向
            int is_read = (desc.flags & DMA_FLAG_READ) != 0;
            
            // 执行数据传输
            void* buf = malloc(desc.length);
            
            if (is_read) {
                // Host -> Device (SSD读取数据)
                pcie->host_dma_read(desc.src_addr, buf, desc.length);
                // 写入SSD内存
                memcpy((void*)desc.dst_addr, buf, desc.length);
                pcie->endpoint.dma_bytes_read += desc.length;
            } else {
                // Device -> Host (SSD写入数据)
                // 从SSD内存读取
                memcpy(buf, (void*)desc.src_addr, desc.length);
                pcie->host_dma_write(desc.dst_addr, buf, desc.length);
                pcie->endpoint.dma_bytes_written += desc.length;
            }
            
            free(buf);
            
            dma->bytes_transferred += desc.length;
            desc_processed++;
            desc_ptr = desc.next_desc;
        }
        
        dma->busy = 0;
        
        // 发送完成中断
        pcie_send_interrupt(pcie, dma->id);
    }
    
    return NULL;
}
```

#### 3.4.4 中断处理
```c
void pcie_send_interrupt(pcie_controller_t* pcie, uint32_t vector) {
    if (pcie->endpoint.msix_enabled) {
        // MSI-X中断
        uint64_t addr = ((uint64_t)pcie->endpoint.msi_addr_high << 32) 
                        | pcie->endpoint.msi_addr_low;
        uint32_t data = pcie->endpoint.msi_data | vector;
        
        // 写入MSI-X地址触发中断
        pcie->host_interrupt(vector);
    } else if (pcie->endpoint.msi_enabled) {
        // MSI中断
        pcie->host_interrupt(pcie->endpoint.msi_data);
    } else {
        // 传统中断
        pcie->host_interrupt(pcie->endpoint.int_pin);
    }
}
```

### 3.5 API接口
```c
// 初始化
int pcie_controller_init(pcie_controller_t* pcie, pcie_config_t* config);
void pcie_controller_destroy(pcie_controller_t* pcie);

// 配置空间访问
uint32_t pcie_config_read32(uint16_t offset);
void pcie_config_write32(uint16_t offset, uint32_t value);

// MMIO访问
uint32_t pcie_mmio_read32(uint64_t addr);
void pcie_mmio_write32(uint64_t addr, uint32_t value);
uint64_t pcie_mmio_read64(uint64_t addr);
void pcie_mmio_write64(uint64_t addr, uint64_t value);

// DMA
int pcie_dma_submit(uint32_t dma_id, uint64_t desc_addr, uint32_t desc_count);
int pcie_dma_poll(uint32_t dma_id);
void pcie_dma_wait(uint32_t dma_id);

// 中断
void pcie_send_interrupt(uint32_t vector);
void pcie_mask_interrupt(uint32_t vector);
void pcie_unmask_interrupt(uint32_t vector);

// 统计
void pcie_get_stats(pcie_stats_t* stats);
```

---

## 4. NVMe协议模块 (NVMeModule)

### 4.1 设计目标
- 完整实现NVMe 1.4协议
- 支持Admin和IO队列
- PRP/SGL数据传输
- Namespace管理

### 4.2 NVMe数据结构

```c
// NVMe Command Structure (64 bytes)
typedef struct nvme_command {
    // CDW 0
    uint16_t opcode;            // 操作码
    uint16_t flags;             // 标志
    uint16_t cid;               // Command ID
    uint16_t nsid;              // Namespace ID
    
    // CDW 2-3
    uint64_t reserved;
    
    // CDW 4-5
    uint64_t mptr;              // Metadata Pointer
    
    // CDW 6-7
    uint64_t prp1;              // PRP Entry 1
    
    // CDW 8-9
    uint64_t prp2;              // PRP Entry 2
    
    // CDW 10-15
    uint32_t cdw10;             // 命令特定
    uint32_t cdw11;
    uint32_t cdw12;
    uint32_t cdw13;
    uint32_t cdw14;
    uint32_t cdw15;
} __attribute__((packed)) nvme_command_t;

// NVMe Completion Queue Entry (16 bytes)
typedef struct nvme_completion {
    uint32_t result;            // DW0 - 命令特定结果
    uint32_t reserved;
    uint16_t sq_head;           // SQ Head Pointer
    uint16_t sq_id;             // SQ Identifier
    uint16_t cid;               // Command ID
    uint16_t status;            // 状态字段
} __attribute__((packed)) nvme_completion_t;

// Submission Queue Entry (same as command)
typedef nvme_command_t sq_entry_t;

// NVMe Queue
typedef struct nvme_queue {
    uint16_t qid;               // Queue ID
    uint16_t size;              // 队列大小 (条目数)
    uint64_t base_addr;         // 基地址 (主机物理地址)
    
    // Doorbell
    uint32_t* tail_doorbell;    // Tail Doorbell (写入)
    uint32_t* head_doorbell;    // Head Doorbell (读取)
    
    // 内部指针
    uint16_t head;              // Head Pointer (下一个要处理的)
    uint16_t tail;              // Tail Pointer (下一个可写入的)
    uint16_t count;             // 当前条目数
    
    // 标志
    uint8_t cq_assoc;           // 关联的CQ ID
    uint8_t priority;           // 优先级
    
    pthread_spinlock_t lock;
} nvme_queue_t;

// Namespace
typedef struct nvme_namespace {
    uint32_t nsid;              // Namespace ID
    uint64_t size;              // 容量 (逻辑块数)
    uint64_t capacity;          // 最大容量
    uint32_t lba_size;          // LBA大小
    uint32_t meta_size;         // 元数据大小
    
    // 性能特征
    uint32_t form_factor;       // 形态
    uint8_t  data_protection;   // 数据保护
    uint8_t  shareable;         // 可共享
    uint8_t  thin_provisioned;  // 精简配置
    
    // L2P管理
    void* l2p_manager;
} nvme_namespace_t;

// Controller Registers (BAR0)
typedef struct nvme_controller_regs {
    // Controller Capabilities
    uint64_t cap;               // 能力
    uint32_t vs;                // 版本
    uint32_t intms;             // Interrupt Mask Set
    uint32_t intmc;             // Interrupt Mask Clear
    uint32_t cc;                // Controller Configuration
    uint32_t reserved;
    uint32_t csts;              // Controller Status
    uint32_t nssr;              // NVM Subsystem Reset
    uint32_t aqa;               // Admin Queue Attributes
    uint64_t asq;               // Admin Submission Queue Base
    uint64_t acq;               // Admin Completion Queue Base
    // ... more registers
} __attribute__((packed)) nvme_controller_regs_t;

// Controller Capability (CAP)
#define NVME_CAP_MQES(cap)      ((cap) & 0xFFFF)        // Max Queue Entries Supported
#define NVME_CAP_CQR(cap)       (((cap) >> 16) & 1)     // Contiguous Queues Required
#define NVME_CAP_AMS(cap)       (((cap) >> 17) & 3)     // Arbitration Mechanism Supported
#define NVME_CAP_TO(cap)        (((cap) >> 24) & 0xFF)  // Timeout
#define NVME_CAP_DSTRD(cap)     (((cap) >> 32) & 0xF)   // Doorbell Stride
#define NVME_CAP_NSSRS(cap)     (((cap) >> 36) & 1)     // NVM Subsystem Reset Supported
#define NVME_CAP_CSS(cap)       (((cap) >> 37) & 0xFF)  // Command Sets Supported
#define NVME_CAP_MPSMIN(cap)    (((cap) >> 48) & 0xF)   // Memory Page Size Minimum
#define NVME_CAP_MPSMAX(cap)    (((cap) >> 52) & 0xF)   // Memory Page Size Maximum

// NVMe Controller
typedef struct nvme_controller {
    // 寄存器
    nvme_controller_regs_t regs;
    
    // 队列
    nvme_queue_t* admin_sq;     // Admin Submission Queue
    nvme_queue_t* admin_cq;     // Admin Completion Queue
    nvme_queue_t** io_sqs;      // IO Submission Queues
    nvme_queue_t** io_cqs;      // IO Completion Queues
    uint32_t num_io_queues;
    
    // Namespace
    nvme_namespace_t** namespaces;
    uint32_t num_namespaces;
    uint32_t max_namespaces;
    
    // 状态
    uint8_t state;              // 控制器状态
    uint8_t ready;              // 就绪标志
    
    // 处理线程
    pthread_t admin_thread;
    pthread_t* io_threads;
    volatile int running;
    
    // 与下层接口
    pcie_controller_t* pcie;
    l2p_manager_t* l2p;
    nand_controller_t* nand;
    
    // 统计
    uint64_t commands_completed;
    uint64_t commands_aborted;
    uint64_t data_read;
    uint64_t data_written;
} nvme_controller_t;

// Controller State
#define NVME_STATE_INIT         0
#define NVME_STATE_READY        1
#define NVME_STATE_RESETTING    2
```

### 4.3 NVMe命令处理

#### 4.3.1 Admin命令
```c
// Admin命令处理
typedef int (*nvme_admin_handler_t)(nvme_controller_t* ctrl, nvme_command_t* cmd, nvme_completion_t* cpl);

static nvme_admin_handler_t g_admin_handlers[256] = {
    [0x00] = nvme_admin_delete_sq,
    [0x01] = nvme_admin_create_sq,
    [0x02] = nvme_admin_get_log_page,
    [0x04] = nvme_admin_delete_cq,
    [0x05] = nvme_admin_create_cq,
    [0x06] = nvme_admin_identify,
    [0x08] = nvme_admin_abort,
    [0x09] = nvme_admin_set_features,
    [0x0A] = nvme_admin_get_features,
    [0x0C] = nvme_admin_async_event_req,
    [0x0D] = nvme_admin_ns_mgmt,
    [0x10] = nvme_admin_fw_commit,
    [0x11] = nvme_admin_fw_img_download,
    // ... more commands
};

int nvme_handle_admin_command(nvme_controller_t* ctrl) {
    // 从Admin SQ取命令
    nvme_queue_t* sq = ctrl->admin_sq;
    nvme_queue_t* cq = ctrl->admin_cq;
    
    pthread_spin_lock(&sq->lock);
    
    while (sq->head != sq->tail) {
        // 读取命令
        uint64_t cmd_addr = sq->base_addr + sq->head * sizeof(nvme_command_t);
        nvme_command_t cmd;
        pcie_dma_read(cmd_addr, &cmd, sizeof(cmd));
        
        // 处理命令
        nvme_completion_t cpl = {0};
        cpl.cid = cmd.cid;
        cpl.sq_id = 0;
        cpl.sq_head = sq->head;
        
        uint8_t opcode = cmd.opcode;
        if (g_admin_handlers[opcode]) {
            int ret = g_admin_handlers[opcode](ctrl, &cmd, &cpl);
            if (ret != 0) {
                cpl.status = (NVME_SC_INTERNAL << 1) | 1;
            }
        } else {
            cpl.status = (NVME_SC_INVALID_OPCODE << 1) | 1;
        }
        
        // 写入CQ
        uint64_t cpl_addr = cq->base_addr + cq->tail * sizeof(nvme_completion_t);
        pcie_dma_write(cpl_addr, &cpl, sizeof(cpl));
        
        // 更新指针
        sq->head = (sq->head + 1) % sq->size;
        cq->tail = (cq->tail + 1) % cq->size;
        
        // 更新Doorbell
        *sq->head_doorbell = sq->head;
        
        // 发送中断
        pcie_send_interrupt(0);
    }
    
    pthread_spin_unlock(&sq->lock);
    
    return 0;
}

// Create CQ命令
int nvme_admin_create_cq(nvme_controller_t* ctrl, nvme_command_t* cmd, nvme_completion_t* cpl) {
    uint16_t qid = cmd->cdw10 & 0xFFFF;
    uint16_t size = ((cmd->cdw10 >> 16) & 0xFFFF) + 1;
    uint8_t pc = (cmd->cdw11 >> 0) & 1;      // Physically Contiguous
    uint8_t ien = (cmd->cdw11 >> 1) & 1;     // Interrupt Enable
    uint16_t iv = (cmd->cdw11 >> 16) & 0xFFFF; // Interrupt Vector
    uint64_t prp1 = cmd->prp1;
    
    if (qid == 0 || qid >= ctrl->num_io_queues) {
        cpl->status = (NVME_SC_INVALID_QID << 1) | 1;
        return -1;
    }
    
    if (ctrl->io_cqs[qid] != NULL) {
        cpl->status = (NVME_SC_INVALID_QID << 1) | 1;
        return -1;
    }
    
    // 创建CQ
    nvme_queue_t* cq = calloc(1, sizeof(nvme_queue_t));
    cq->qid = qid;
    cq->size = size;
    cq->base_addr = prp1;
    cq->head = 0;
    cq->tail = 0;
    cq->count = 0;
    
    // 设置Doorbell地址
    uint64_t bar0_base = ctrl->pcie->endpoint.bars[0].base_addr;
    cq->tail_doorbell = (uint32_t*)(bar0_base + 0x1000 + (2 * qid + 1) * (4 << NVME_CAP_DSTRD(ctrl->regs.cap)));
    cq->head_doorbell = NULL;  // CQ只有Tail Doorbell
    
    pthread_spin_init(&cq->lock, PTHREAD_PROCESS_PRIVATE);
    
    ctrl->io_cqs[qid] = cq;
    
    return 0;
}

// Create SQ命令
int nvme_admin_create_sq(nvme_controller_t* ctrl, nvme_command_t* cmd, nvme_completion_t* cpl) {
    uint16_t qid = cmd->cdw10 & 0xFFFF;
    uint16_t size = ((cmd->cdw10 >> 16) & 0xFFFF) + 1;
    uint8_t pc = (cmd->cdw11 >> 0) & 1;
    uint8_t prio = (cmd->cdw11 >> 1) & 0x3;
    uint16_t cqid = (cmd->cdw11 >> 16) & 0xFFFF;
    uint64_t prp1 = cmd->prp1;
    
    if (qid == 0 || qid >= ctrl->num_io_queues) {
        cpl->status = (NVME_SC_INVALID_QID << 1) | 1;
        return -1;
    }
    
    if (ctrl->io_sqs[qid] != NULL) {
        cpl->status = (NVME_SC_INVALID_QID << 1) | 1;
        return -1;
    }
    
    if (ctrl->io_cqs[cqid] == NULL) {
        cpl->status = (NVME_SC_INVALID_CQID << 1) | 1;
        return -1;
    }
    
    // 创建SQ
    nvme_queue_t* sq = calloc(1, sizeof(nvme_queue_t));
    sq->qid = qid;
    sq->size = size;
    sq->base_addr = prp1;
    sq->head = 0;
    sq->tail = 0;
    sq->count = 0;
    sq->cq_assoc = cqid;
    sq->priority = prio;
    
    // 设置Doorbell地址
    uint64_t bar0_base = ctrl->pcie->endpoint.bars[0].base_addr;
    sq->tail_doorbell = (uint32_t*)(bar0_base + 0x1000 + (2 * qid) * (4 << NVME_CAP_DSTRD(ctrl->regs.cap)));
    sq->head_doorbell = NULL;  // SQ只有Head Doorbell (由控制器写)
    
    pthread_spin_init(&sq->lock, PTHREAD_PROCESS_PRIVATE);
    
    ctrl->io_sqs[qid] = sq;
    
    return 0;
}
```

#### 4.3.2 IO命令
```c
// IO命令处理
typedef int (*nvme_io_handler_t)(nvme_controller_t* ctrl, nvme_namespace_t* ns, 
                                  nvme_command_t* cmd, nvme_completion_t* cpl);

static nvme_io_handler_t g_io_handlers[256] = {
    [0x00] = nvme_io_flush,
    [0x01] = nvme_io_write,
    [0x02] = nvme_io_read,
    // ... more IO commands
};

// Read命令
int nvme_io_read(nvme_controller_t* ctrl, nvme_namespace_t* ns, 
                 nvme_command_t* cmd, nvme_completion_t* cpl) {
    // 解析命令参数
    uint64_t slba = ((uint64_t)cmd->cdw11 << 32) | cmd->cdw10;  // 起始LBA
    uint32_t nlb = (cmd->cdw12 & 0xFFFF) + 1;                   // 块数
    
    uint64_t prp1 = cmd->prp1;
    uint64_t prp2 = cmd->prp2;
    
    // 验证范围
    if (slba + nlb > ns->size) {
        cpl->status = (NVME_SC_LBA_RANGE << 1) | 1;
        return -1;
    }
    
    // 分配缓冲区
    uint32_t buf_size = nlb * ns->lba_size;
    uint8_t* buf = ddr_alloc_contiguous(buf_size, ns->lba_size);
    
    // 读取每个LBA
    for (uint32_t i = 0; i < nlb; i++) {
        uint64_t lba = slba + i;
        
        // L2P查找
        l2p_entry_t entry;
        int ret = l2p_lookup(ctrl->l2p, lba, &entry);
        
        if (ret != 0 || !(entry.flags & L2P_FLAG_VALID)) {
            // 未映射区域，返回0
            memset(buf + i * ns->lba_size, 0, ns->lba_size);
            continue;
        }
        
        // 从NAND读取
        uint64_t ppa = entry.ppa;
        uint8_t* page_buf = ddr_alloc(NAND_PAGE_SIZE, NAND_PAGE_SIZE);
        
        ret = nand_read_page(ppa, page_buf);
        if (ret != 0) {
            cpl->status = (NVME_SC_READ_ERROR << 1) | 1;
            ddr_free(page_buf);
            ddr_free(buf);
            return -1;
        }
        
        // 提取LBA数据 (考虑页内偏移)
        uint32_t sector_in_page = lba % NAND_SECTORS_PER_PAGE;
        memcpy(buf + i * ns->lba_size,
               page_buf + sector_in_page * ns->lba_size,
               ns->lba_size);
        
        ddr_free(page_buf);
    }
    
    // 通过DMA传输到主机
    nvme_dma_xfer(ctrl, prp1, prp2, buf, buf_size, DMA_DIR_TO_HOST);
    
    ddr_free(buf);
    
    ctrl->data_read += buf_size;
    
    return 0;
}

// Write命令
int nvme_io_write(nvme_controller_t* ctrl, nvme_namespace_t* ns,
                  nvme_command_t* cmd, nvme_completion_t* cpl) {
    // 解析命令参数
    uint64_t slba = ((uint64_t)cmd->cdw11 << 32) | cmd->cdw10;
    uint32_t nlb = (cmd->cdw12 & 0xFFFF) + 1;
    
    uint64_t prp1 = cmd->prp1;
    uint64_t prp2 = cmd->prp2;
    
    // 验证范围
    if (slba + nlb > ns->size) {
        cpl->status = (NVME_SC_LBA_RANGE << 1) | 1;
        return -1;
    }
    
    // 分配缓冲区并从主机接收数据
    uint32_t buf_size = nlb * ns->lba_size;
    uint8_t* buf = ddr_alloc_contiguous(buf_size, ns->lba_size);
    
    nvme_dma_xfer(ctrl, prp1, prp2, buf, buf_size, DMA_DIR_FROM_HOST);
    
    // 写入每个LBA
    for (uint32_t i = 0; i < nlb; i++) {
        uint64_t lba = slba + i;
        
        // 分配新的物理页
        uint64_t new_ppa;
        int ret = ppa_alloc(&new_ppa, 1, PAGE_TYPE_DATA);
        if (ret != 0) {
            cpl->status = (NVME_SC_NO_RESOURCE << 1) | 1;
            ddr_free(buf);
            return -1;
        }
        
        // 读取原页 (用于合并)
        l2p_entry_t old_entry;
        ret = l2p_lookup(ctrl->l2p, lba, &old_entry);
        
        uint8_t* page_buf = ddr_alloc(NAND_PAGE_SIZE, NAND_PAGE_SIZE);
        
        if (ret == 0 && (old_entry.flags & L2P_FLAG_VALID)) {
            // 读取原页数据
            nand_read_page(old_entry.ppa, page_buf);
        } else {
            memset(page_buf, 0, NAND_PAGE_SIZE);
        }
        
        // 更新页内数据
        uint32_t sector_in_page = lba % NAND_SECTORS_PER_PAGE;
        memcpy(page_buf + sector_in_page * ns->lba_size,
               buf + i * ns->lba_size,
               ns->lba_size);
        
        // 写入新页
        ret = nand_write_page(new_ppa, page_buf, NULL);
        if (ret != 0) {
            cpl->status = (NVME_SC_WRITE_FAULT << 1) | 1;
            ddr_free(page_buf);
            ddr_free(buf);
            return -1;
        }
        
        ddr_free(page_buf);
        
        // 更新L2P
        l2p_update(ctrl->l2p, lba, new_ppa, L2P_FLAG_VALID);
    }
    
    ddr_free(buf);
    
    ctrl->data_written += buf_size;
    
    return 0;
}
```

#### 4.3.3 PRP处理
```c
// PRP传输
int nvme_prp_xfer(nvme_controller_t* ctrl, uint64_t prp1, uint64_t prp2,
                  void* buf, size_t len, int dir) {
    uint8_t* ptr = buf;
    size_t remaining = len;
    
    // PRP1总是指向第一个页
    uint64_t prp = prp1;
    
    while (remaining > 0) {
        size_t chunk_size = MIN(remaining, NVME_PAGE_SIZE);
        
        if (dir == DMA_DIR_TO_HOST) {
            pcie_dma_write(prp, ptr, chunk_size);
        } else {
            pcie_dma_read(prp, ptr, chunk_size);
        }
        
        ptr += chunk_size;
        remaining -= chunk_size;
        
        if (remaining == 0) break;
        
        // 获取下一个PRP
        if (prp == prp1) {
            // 第一个传输完成，使用PRP2
            prp = prp2;
            
            // 检查PRP2是否是指向PRP List的指针
            if (len > 2 * NVME_PAGE_SIZE) {
                // PRP2是PRP List的地址
                // 读取PRP List并处理
                uint64_t prp_list[512];
                pcie_dma_read(prp2, prp_list, sizeof(prp_list));
                
                for (int i = 0; i < 512 && remaining > 0; i++) {
                    prp = prp_list[i];
                    chunk_size = MIN(remaining, NVME_PAGE_SIZE);
                    
                    if (dir == DMA_DIR_TO_HOST) {
                        pcie_dma_write(prp, ptr, chunk_size);
                    } else {
                        pcie_dma_read(prp, ptr, chunk_size);
                    }
                    
                    ptr += chunk_size;
                    remaining -= chunk_size;
                }
                break;
            }
        } else {
            // PRP2直接指向第二个页
            // 没有更多数据了
            break;
        }
    }
    
    return 0;
}
```

### 4.4 API接口
```c
// 初始化
int nvme_controller_init(nvme_controller_t* ctrl, nvme_config_t* config);
void nvme_controller_destroy(nvme_controller_t* ctrl);

// 队列管理
int nvme_create_io_queue(uint16_t qid, uint16_t size, uint8_t type);
int nvme_delete_io_queue(uint16_t qid, uint8_t type);

// Namespace管理
int nvme_create_namespace(uint32_t nsid, uint64_t size, uint32_t lba_size);
int nvme_delete_namespace(uint32_t nsid);
nvme_namespace_t* nvme_get_namespace(uint32_t nsid);

// 命令处理
int nvme_submit_admin_command(nvme_command_t* cmd);
int nvme_submit_io_command(uint16_t qid, nvme_command_t* cmd);

// 状态
int nvme_controller_enable(void);
int nvme_controller_disable(void);
int nvme_controller_reset(void);

// 统计
void nvme_get_stats(nvme_stats_t* stats);
```

---

## 5. 模块交互图

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Host System                                  │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                 │
│  │   fio/dd    │  │  nvme-cli   │  │  Block Dev  │                 │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘                 │
└─────────┼────────────────┼────────────────┼────────────────────────┘
          │                │                │
          └────────────────┴────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      Linux NVMe Driver                               │
│                      (Host Side)                                     │
└─────────────────────────────────────────────────────────────────────┘
                           │
                           │ PCIe TLPs
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    PCIeController                                    │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                 │
│  │ Config Space│  │  BAR0/1     │  │    DMA      │                 │
│  │   (4KB)     │  │  (MMIO)     │  │   Engine    │                 │
│  └─────────────┘  └─────────────┘  └─────────────┘                 │
└─────────────────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│                     NVMeModule                                       │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                 │
│  │ Admin Queue │  │  IO Queues  │  │  Namespace  │                 │
│  │  Process    │  │  Process    │  │   Manager   │                 │
│  └─────────────┘  └─────────────┘  └─────────────┘                 │
└─────────────────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      Core Layer                                      │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐│
│  │   L2P       │  │   FTL       │  │   DDR       │  │    GC       ││
│  │  Manager    │  │   Core      │  │  Manager    │  │   Manager   ││
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘│
└─────────────────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    NANDSimulator                                     │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐                 │
│  │  Channel 0  │  │  Channel 1  │  │  Channel 7  │                 │
│  │ ┌─────────┐ │  │ ┌─────────┐ │  │ ┌─────────┐ │                 │
│  │ │Die/Plane│ │  │ │Die/Plane│ │  │ │Die/Plane│ │                 │
│  │ │ /Block  │ │  │ │ /Block  │ │  │ │ /Block  │ │                 │
│  │ └─────────┘ │  │ └─────────┘ │  │ └─────────┘ │                 │
│  └─────────────┘  └─────────────┘  └─────────────┘                 │
└─────────────────────────────────────────────────────────────────────┘
```

---

**文档版本**: 1.0  
**创建日期**: 2026-02-06  
**状态**: 详细设计完成
