/**
 * @file ddr_manager.c
 * @brief SSD Simulator - DDR Memory Manager Implementation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <pthread.h>
#include "ddr_manager.h"

/* 块大小定义 */
static const size_t g_block_sizes[MEM_BLOCK_NUM] = {
    4096,       /* 4K */
    16384,      /* 16K */
    65536,      /* 64K */
    262144,     /* 256K */
    1048576,    /* 1M */
    4194304     /* 4M */
};

/* 内存块头 */
typedef struct mem_block {
    uint32_t magic;             /* 魔数 */
    mem_block_size_t size_class; /* 大小等级 */
    uint32_t flags;
    uint32_t ref_count;
    size_t size;
    
    /* 分配信息 */
    const char* alloc_file;
    uint32_t alloc_line;
    uint64_t alloc_time;
    
    /* 链表 */
    struct mem_block* next;
    struct mem_block* prev;
} mem_block_t;

#define MEM_BLOCK_MAGIC     0x44445242  /* "DDRB" */

/* Slab分配器 */
typedef struct slab_allocator {
    mem_block_size_t size_class;
    size_t block_size;
    size_t block_count;
    
    void* memory_pool;
    mem_block_t* free_list;
    uint32_t free_count;
    uint32_t used_count;
    
    pthread_spinlock_t lock;
    
    /* 统计 */
    uint64_t allocs;
    uint64_t frees;
    uint64_t alloc_failures;
} slab_allocator_t;

/* DDR管理器 */
typedef struct ddr_manager {
    uint64_t total_capacity;
    uint64_t used_capacity;
    uint64_t peak_usage;
    
    /* Slab分配器 */
    slab_allocator_t slabs[MEM_BLOCK_NUM];
    
    /* 缓冲区池 */
    buffer_pool_t data_pool;
    buffer_pool_t meta_pool;
    buffer_pool_t map_pool;
    
    /* 大内存分配 */
    pthread_mutex_t big_lock;
    
    /* 统计 */
    uint64_t total_allocs;
    uint64_t total_frees;
    uint64_t alloc_failures;
} ddr_manager_t;

static ddr_manager_t g_ddr = {0};

/* 前向声明 */
static int slab_init(slab_allocator_t* slab, mem_block_size_t size_class, uint32_t count);
static void* slab_alloc(slab_allocator_t* slab);
static void slab_free(slab_allocator_t* slab, void* ptr);
static mem_block_size_t size_to_class(size_t size);
static inline void* block_to_ptr(mem_block_t* block);
static inline mem_block_t* ptr_to_block(void* ptr);

int ddr_manager_init(uint64_t capacity) {
    if (g_ddr.total_capacity > 0) {
        return -EALREADY;
    }
    
    g_ddr.total_capacity = capacity;
    g_ddr.used_capacity = 0;
    g_ddr.peak_usage = 0;
    
    /* 初始化Slab分配器 */
    /* 4K块: 大量 */
    slab_init(&g_ddr.slabs[MEM_BLOCK_4K], MEM_BLOCK_4K, 4096);
    /* 16K块: NAND页 */
    slab_init(&g_ddr.slabs[MEM_BLOCK_16K], MEM_BLOCK_16K, 2048);
    /* 64K块 */
    slab_init(&g_ddr.slabs[MEM_BLOCK_64K], MEM_BLOCK_64K, 512);
    /* 256K块: 映射表 */
    slab_init(&g_ddr.slabs[MEM_BLOCK_256K], MEM_BLOCK_256K, 256);
    /* 1M块 */
    slab_init(&g_ddr.slabs[MEM_BLOCK_1M], MEM_BLOCK_1M, 64);
    /* 4M块 */
    slab_init(&g_ddr.slabs[MEM_BLOCK_4M], MEM_BLOCK_4M, 16);
    
    pthread_mutex_init(&g_ddr.big_lock, NULL);
    
    /* 初始化缓冲区池 */
    memset(&g_ddr.data_pool, 0, sizeof(buffer_pool_t));
    g_ddr.data_pool.name = "data";
    g_ddr.data_pool.buffer_size = 16384;
    g_ddr.data_pool.num_buffers = 1024;
    pthread_spin_init(&g_ddr.data_pool.lock, PTHREAD_PROCESS_PRIVATE);
    
    memset(&g_ddr.meta_pool, 0, sizeof(buffer_pool_t));
    g_ddr.meta_pool.name = "meta";
    g_ddr.meta_pool.buffer_size = 4096;
    g_ddr.meta_pool.num_buffers = 2048;
    pthread_spin_init(&g_ddr.meta_pool.lock, PTHREAD_PROCESS_PRIVATE);
    
    memset(&g_ddr.map_pool, 0, sizeof(buffer_pool_t));
    g_ddr.map_pool.name = "map";
    g_ddr.map_pool.buffer_size = 262144;
    g_ddr.map_pool.num_buffers = 128;
    pthread_spin_init(&g_ddr.map_pool.lock, PTHREAD_PROCESS_PRIVATE);
    
    printf("[DDR] Manager initialized: %lu MB\n", capacity / (1024 * 1024));
    return 0;
}

void ddr_manager_destroy(void) {
    if (g_ddr.total_capacity == 0) return;
    
    /* 销毁Slab分配器 */
    for (int i = 0; i < MEM_BLOCK_NUM; i++) {
        slab_allocator_t* slab = &g_ddr.slabs[i];
        pthread_spin_destroy(&slab->lock);
        free(slab->memory_pool);
    }
    
    pthread_mutex_destroy(&g_ddr.big_lock);
    pthread_spin_destroy(&g_ddr.data_pool.lock);
    pthread_spin_destroy(&g_ddr.meta_pool.lock);
    pthread_spin_destroy(&g_ddr.map_pool.lock);
    
    memset(&g_ddr, 0, sizeof(g_ddr));
    printf("[DDR] Manager destroyed\n");
}

void* ddr_alloc(size_t size, size_t align) {
    return ddr_alloc_flags(size, align, 0);
}

void* ddr_alloc_flags(size_t size, size_t align, uint32_t flags) {
    if (size == 0) return NULL;
    
    /* 对齐 */
    if (align < 64) align = 64;
    size = (size + align - 1) & ~(align - 1);
    
    void* ptr = NULL;
    
    /* 选择合适的分配策略 */
    mem_block_size_t size_class = size_to_class(size);
    
    if (size_class < MEM_BLOCK_NUM) {
        /* 从Slab分配 */
        ptr = slab_alloc(&g_ddr.slabs[size_class]);
    } else {
        /* 大内存: 直接malloc */
        pthread_mutex_lock(&g_ddr.big_lock);
        
        size_t total_size = sizeof(mem_block_t) + size + align;
        mem_block_t* block = aligned_alloc(align, total_size);
        
        if (block) {
            block->magic = MEM_BLOCK_MAGIC;
            block->size_class = MEM_BLOCK_NUM; /* 标记为大内存 */
            block->flags = flags;
            block->ref_count = 1;
            block->size = size;
            block->alloc_file = NULL;
            block->alloc_line = 0;
            
            ptr = block_to_ptr(block);
            
            /* 对齐 */
            uintptr_t addr = (uintptr_t)ptr;
            addr = (addr + align - 1) & ~(align - 1);
            ptr = (void*)addr;
        }
        
        pthread_mutex_unlock(&g_ddr.big_lock);
    }
    
    if (ptr) {
        if (flags & MEM_FLAG_ZERO) {
            memset(ptr, 0, size);
        }
        
        g_ddr.total_allocs++;
        g_ddr.used_capacity += size;
        if (g_ddr.used_capacity > g_ddr.peak_usage) {
            g_ddr.peak_usage = g_ddr.used_capacity;
        }
    } else {
        g_ddr.alloc_failures++;
    }
    
    return ptr;
}

void ddr_free(void* ptr) {
    if (!ptr) return;
    
    mem_block_t* block = ptr_to_block(ptr);
    
    /* 验证 */
    if (block->magic != MEM_BLOCK_MAGIC) {
        fprintf(stderr, "[DDR] Error: Invalid memory block freed!\n");
        return;
    }
    
    /* 引用计数 */
    if (__sync_sub_and_fetch(&block->ref_count, 1) > 0) {
        return;
    }
    
    g_ddr.total_frees++;
    g_ddr.used_capacity -= block->size;
    
    if (block->size_class < MEM_BLOCK_NUM) {
        slab_free(&g_ddr.slabs[block->size_class], ptr);
    } else {
        /* 大内存 */
        pthread_mutex_lock(&g_ddr.big_lock);
        free(block);
        pthread_mutex_unlock(&g_ddr.big_lock);
    }
}

void* ddr_realloc(void* ptr, size_t new_size) {
    if (!ptr) return ddr_alloc(new_size, 64);
    if (new_size == 0) {
        ddr_free(ptr);
        return NULL;
    }
    
    mem_block_t* block = ptr_to_block(ptr);
    if (block->magic != MEM_BLOCK_MAGIC) {
        return NULL;
    }
    
    /* 分配新内存 */
    void* new_ptr = ddr_alloc(new_size, 64);
    if (!new_ptr) return NULL;
    
    /* 复制数据 */
    size_t copy_size = (new_size < block->size) ? new_size : block->size;
    memcpy(new_ptr, ptr, copy_size);
    
    /* 释放旧内存 */
    ddr_free(ptr);
    
    return new_ptr;
}

void ddr_ref(void* ptr) {
    if (!ptr) return;
    
    mem_block_t* block = ptr_to_block(ptr);
    if (block->magic == MEM_BLOCK_MAGIC) {
        __sync_fetch_and_add(&block->ref_count, 1);
    }
}

void ddr_unref(void* ptr) {
    ddr_free(ptr);
}

data_buffer_t* ddr_buffer_acquire(const char* pool_name) {
    buffer_pool_t* pool = NULL;
    
    if (strcmp(pool_name, "data") == 0) {
        pool = &g_ddr.data_pool;
    } else if (strcmp(pool_name, "meta") == 0) {
        pool = &g_ddr.meta_pool;
    } else if (strcmp(pool_name, "map") == 0) {
        pool = &g_ddr.map_pool;
    } else {
        return NULL;
    }
    
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
        buf->ref_count = 1;
        pool->hits++;
        return buf;
    }
    
    pthread_spin_unlock(&pool->lock);
    
    /* 池耗尽，从Slab分配 */
    buf = ddr_alloc(sizeof(data_buffer_t) + pool->buffer_size, 64);
    if (buf) {
        buf->virtual_addr = (char*)buf + sizeof(data_buffer_t);
        buf->physical_addr = ddr_virt_to_phys(buf->virtual_addr);
        buf->size = pool->buffer_size;
        buf->ref_count = 1;
        buf->next = NULL;
        buf->prev = NULL;
    }
    
    pool->misses++;
    return buf;
}

void ddr_buffer_release(data_buffer_t* buf) {
    if (!buf) return;
    
    if (__sync_sub_and_fetch(&buf->ref_count, 1) > 0) {
        return;
    }
    
    /* 确定属于哪个池 */
    buffer_pool_t* pool = NULL;
    if (buf->size == 16384) {
        pool = &g_ddr.data_pool;
    } else if (buf->size == 4096) {
        pool = &g_ddr.meta_pool;
    } else if (buf->size == 262144) {
        pool = &g_ddr.map_pool;
    }
    
    if (pool) {
        pthread_spin_lock(&pool->lock);
        buf->next = pool->free_list;
        buf->prev = NULL;
        if (pool->free_list) {
            pool->free_list->prev = buf;
        }
        pool->free_list = buf;
        pool->free_count++;
        pthread_spin_unlock(&pool->lock);
    } else {
        ddr_free(buf);
    }
}

uint64_t ddr_virt_to_phys(void* virt) {
    /* 简单线性映射 */
    return (uint64_t)virt;
}

void* ddr_phys_to_virt(uint64_t phys) {
    return (void*)phys;
}

void ddr_get_stats(ddr_stats_t* stats) {
    if (!stats) return;
    
    memset(stats, 0, sizeof(*stats));
    
    stats->total_allocs = g_ddr.total_allocs;
    stats->total_frees = g_ddr.total_frees;
    stats->current_used = g_ddr.used_capacity;
    stats->peak_used = g_ddr.peak_usage;
    stats->alloc_failures = g_ddr.alloc_failures;
    
    for (int i = 0; i < MEM_BLOCK_NUM; i++) {
        stats->slab_stats[i].allocs = g_ddr.slabs[i].allocs;
        stats->slab_stats[i].frees = g_ddr.slabs[i].frees;
        stats->slab_stats[i].current = g_ddr.slabs[i].used_count;
    }
}

void ddr_dump_usage(void) {
    ddr_stats_t stats;
    ddr_get_stats(&stats);
    
    printf("\n[DDR] Memory Usage:\n");
    printf("  Total allocs: %lu\n", stats.total_allocs);
    printf("  Total frees:  %lu\n", stats.total_frees);
    printf("  Current used: %lu MB\n", stats.current_used / (1024 * 1024));
    printf("  Peak usage:   %lu MB\n", stats.peak_used / (1024 * 1024));
    printf("  Failures:     %lu\n", stats.alloc_failures);
    
    printf("\n  Slab usage:\n");
    for (int i = 0; i < MEM_BLOCK_NUM; i++) {
        printf("    %4zuKB: %lu / %lu\n",
               g_block_sizes[i] / 1024,
               stats.slab_stats[i].current,
               stats.slab_stats[i].allocs);
    }
}

/* Slab分配器实现 */
static int slab_init(slab_allocator_t* slab, mem_block_size_t size_class, uint32_t count) {
    slab->size_class = size_class;
    slab->block_size = g_block_sizes[size_class];
    slab->block_count = count;
    slab->free_count = count;
    slab->used_count = 0;
    
    /* 分配内存池 */
    size_t total_size = sizeof(mem_block_t) + slab->block_size;
    slab->memory_pool = calloc(count, total_size);
    if (!slab->memory_pool) {
        return -ENOMEM;
    }
    
    /* 初始化空闲链表 */
    mem_block_t* prev = NULL;
    for (uint32_t i = 0; i < count; i++) {
        mem_block_t* block = (mem_block_t*)((char*)slab->memory_pool + i * total_size);
        block->magic = MEM_BLOCK_MAGIC;
        block->size_class = size_class;
        block->size = slab->block_size;
        block->next = NULL;
        block->prev = prev;
        
        if (prev) {
            prev->next = block;
        } else {
            slab->free_list = block;
        }
        prev = block;
    }
    
    pthread_spin_init(&slab->lock, PTHREAD_PROCESS_PRIVATE);
    
    return 0;
}

static void* slab_alloc(slab_allocator_t* slab) {
    pthread_spin_lock(&slab->lock);
    
    mem_block_t* block = slab->free_list;
    if (!block) {
        pthread_spin_unlock(&slab->lock);
        slab->alloc_failures++;
        return NULL;
    }
    
    /* 从链表移除 */
    slab->free_list = block->next;
    if (slab->free_list) {
        slab->free_list->prev = NULL;
    }
    slab->free_count--;
    slab->used_count++;
    
    pthread_spin_unlock(&slab->lock);
    
    block->flags = 0;
    block->ref_count = 1;
    block->next = NULL;
    block->prev = NULL;
    
    slab->allocs++;
    
    return block_to_ptr(block);
}

static void slab_free(slab_allocator_t* slab, void* ptr) {
    mem_block_t* block = ptr_to_block(ptr);
    
    pthread_spin_lock(&slab->lock);
    
    /* 加入空闲链表头部 */
    block->next = slab->free_list;
    block->prev = NULL;
    if (slab->free_list) {
        slab->free_list->prev = block;
    }
    slab->free_list = block;
    slab->free_count++;
    slab->used_count--;
    
    pthread_spin_unlock(&slab->lock);
    
    slab->frees++;
}

static mem_block_size_t size_to_class(size_t size) {
    for (int i = 0; i < MEM_BLOCK_NUM; i++) {
        if (size <= g_block_sizes[i]) {
            return i;
        }
    }
    return MEM_BLOCK_NUM; /* 大内存 */
}

static inline void* block_to_ptr(mem_block_t* block) {
    return (char*)block + sizeof(mem_block_t);
}

static inline mem_block_t* ptr_to_block(void* ptr) {
    return (mem_block_t*)((char*)ptr - sizeof(mem_block_t));
}
