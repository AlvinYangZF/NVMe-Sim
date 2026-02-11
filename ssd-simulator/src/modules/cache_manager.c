/**
 * @file cache_manager.c
 * @brief SSD Simulator - Cache Manager Implementation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include "cache_manager.h"
#include "ddr_manager.h"

/* 缓存条目 */
typedef struct cache_entry {
    uint64_t lba;               /* 逻辑块地址 */
    void* data;                 /* 数据指针 */
    cache_entry_state_t state;  /* 状态 */
    uint64_t last_access;       /* 最后访问时间 */
    uint32_t access_count;      /* 访问计数 */
    
    /* LRU链表 */
    struct cache_entry* lru_next;
    struct cache_entry* lru_prev;
    
    /* Hash链表 */
    struct cache_entry* hash_next;
} cache_entry_t;

/* 缓存管理器 */
struct cache_manager {
    uint32_t capacity;          /* 容量（条目数） */
    uint32_t page_size;         /* 页大小 */
    uint32_t used;              /* 已用条目数 */
    
    /* 缓存条目数组 */
    cache_entry_t* entries;
    
    /* Hash表 */
    cache_entry_t** hash_table;
    uint32_t hash_size;
    
    /* LRU链表 */
    cache_entry_t* lru_head;
    cache_entry_t* lru_tail;
    
    pthread_rwlock_t lock;
    
    /* 统计 */
    cache_stats_t stats;
};

/* 前向声明 */
static uint32_t hash_lba(uint64_t lba, uint32_t hash_size);
static cache_entry_t* cache_lookup(cache_manager_t* cache, uint64_t lba);
static cache_entry_t* cache_alloc_entry(cache_manager_t* cache);
static void cache_update_lru(cache_manager_t* cache, cache_entry_t* entry);
static void cache_remove_lru(cache_manager_t* cache, cache_entry_t* entry);
static inline uint64_t get_time_ns(void);

cache_manager_t* cache_manager_create(uint32_t capacity, uint32_t page_size) {
    if (capacity == 0 || page_size == 0) return NULL;
    
    cache_manager_t* cache = calloc(1, sizeof(cache_manager_t));
    if (!cache) return NULL;
    
    cache->capacity = capacity;
    cache->page_size = page_size;
    cache->used = 0;
    
    /* 分配条目数组 */
    cache->entries = calloc(capacity, sizeof(cache_entry_t));
    if (!cache->entries) {
        free(cache);
        return NULL;
    }
    
    /* 为每个条目分配数据缓冲区 */
    for (uint32_t i = 0; i < capacity; i++) {
        cache->entries[i].data = ddr_alloc(page_size, page_size);
        if (!cache->entries[i].data) {
            for (uint32_t j = 0; j < i; j++) {
                ddr_free(cache->entries[j].data);
            }
            free(cache->entries);
            free(cache);
            return NULL;
        }
        cache->entries[i].state = CACHE_ENTRY_FREE;
    }
    
    /* 分配Hash表 */
    cache->hash_size = capacity * 2;
    cache->hash_table = calloc(cache->hash_size, sizeof(cache_entry_t*));
    if (!cache->hash_table) {
        for (uint32_t i = 0; i < capacity; i++) {
            ddr_free(cache->entries[i].data);
        }
        free(cache->entries);
        free(cache);
        return NULL;
    }
    
    pthread_rwlock_init(&cache->lock, NULL);
    
    printf("[Cache] Manager created: %u pages, %u KB each\n", 
           capacity, page_size / 1024);
    
    return cache;
}

void cache_manager_destroy(cache_manager_t* cache) {
    if (!cache) return;
    
    /* 刷新所有脏数据 */
    cache_flush(cache, -1);
    
    pthread_rwlock_destroy(&cache->lock);
    
    /* 释放数据缓冲区 */
    for (uint32_t i = 0; i < cache->capacity; i++) {
        ddr_free(cache->entries[i].data);
    }
    
    free(cache->hash_table);
    free(cache->entries);
    free(cache);
    
    printf("[Cache] Manager destroyed\n");
}

int cache_read(cache_manager_t* cache, uint64_t lba, void* buf) {
    if (!cache || !buf) return -EINVAL;
    
    pthread_rwlock_rdlock(&cache->lock);
    
    cache_entry_t* entry = cache_lookup(cache, lba);
    
    if (entry && entry->state != CACHE_ENTRY_FREE) {
        /* 缓存命中 */
        memcpy(buf, entry->data, cache->page_size);
        entry->last_access = get_time_ns();
        __sync_fetch_and_add(&entry->access_count, 1);
        cache_update_lru(cache, entry);
        
        pthread_rwlock_unlock(&cache->lock);
        
        __sync_fetch_and_add(&cache->stats.read_hits, 1);
        return 0;
    }
    
    pthread_rwlock_unlock(&cache->lock);
    
    __sync_fetch_and_add(&cache->stats.read_misses, 1);
    return -ENOENT;
}

int cache_write(cache_manager_t* cache, uint64_t lba, void* buf) {
    if (!cache || !buf) return -EINVAL;
    
    pthread_rwlock_wrlock(&cache->lock);
    
    cache_entry_t* entry = cache_lookup(cache, lba);
    
    if (!entry) {
        /* 分配新条目 */
        entry = cache_alloc_entry(cache);
        if (!entry) {
            pthread_rwlock_unlock(&cache->lock);
            return -ENOMEM;
        }
        
        /* 初始化条目 */
        entry->lba = lba;
        entry->state = CACHE_ENTRY_VALID;
        entry->last_access = get_time_ns();
        entry->access_count = 1;
        
        /* 加入Hash表 */
        uint32_t hash = hash_lba(lba, cache->hash_size);
        entry->hash_next = cache->hash_table[hash];
        cache->hash_table[hash] = entry;
        
        cache->used++;
    }
    
    /* 写入数据 */
    memcpy(entry->data, buf, cache->page_size);
    entry->state = CACHE_ENTRY_DIRTY;
    entry->last_access = get_time_ns();
    
    cache_update_lru(cache, entry);
    
    pthread_rwlock_unlock(&cache->lock);
    
    if (entry->access_count == 1) {
        __sync_fetch_and_add(&cache->stats.write_misses, 1);
    } else {
        __sync_fetch_and_add(&cache->stats.write_hits, 1);
    }
    
    return 0;
}

int cache_invalidate(cache_manager_t* cache, uint64_t lba) {
    if (!cache) return -EINVAL;
    
    pthread_rwlock_wrlock(&cache->lock);
    
    cache_entry_t* entry = cache_lookup(cache, lba);
    if (entry && entry->state != CACHE_ENTRY_FREE) {
        /* 如果脏，先刷新 */
        if (entry->state == CACHE_ENTRY_DIRTY) {
            /* 实际实现会写入NAND */
            entry->state = CACHE_ENTRY_VALID;
        }
        
        entry->state = CACHE_ENTRY_FREE;
        entry->lba = 0;
        
        /* 从Hash表移除 */
        uint32_t hash = hash_lba(lba, cache->hash_size);
        cache_entry_t** pp = &cache->hash_table[hash];
        while (*pp) {
            if (*pp == entry) {
                *pp = entry->hash_next;
                break;
            }
            pp = &(*pp)->hash_next;
        }
        
        /* 从LRU链表移除 */
        cache_remove_lru(cache, entry);
        
        cache->used--;
    }
    
    pthread_rwlock_unlock(&cache->lock);
    
    return 0;
}

uint32_t cache_flush(cache_manager_t* cache, int64_t lba) {
    if (!cache) return 0;
    
    uint32_t flushed = 0;
    
    pthread_rwlock_wrlock(&cache->lock);
    
    if (lba < 0) {
        /* 刷新所有脏数据 */
        for (uint32_t i = 0; i < cache->capacity; i++) {
            cache_entry_t* entry = &cache->entries[i];
            if (entry->state == CACHE_ENTRY_DIRTY) {
                /* 实际实现会写入NAND */
                entry->state = CACHE_ENTRY_VALID;
                flushed++;
            }
        }
    } else {
        /* 刷新指定LBA */
        cache_entry_t* entry = cache_lookup(cache, lba);
        if (entry && entry->state == CACHE_ENTRY_DIRTY) {
            entry->state = CACHE_ENTRY_VALID;
            flushed++;
        }
    }
    
    pthread_rwlock_unlock(&cache->lock);
    
    __sync_fetch_and_add(&cache->stats.flushes, flushed);
    
    return flushed;
}

uint32_t cache_prefetch(cache_manager_t* cache, uint64_t lba, uint32_t count) {
    if (!cache || count == 0) return 0;
    
    /* 简化实现：仅标记需要预读的LBA */
    /* 实际实现会从NAND读取数据到缓存 */
    
    return 0;
}

void cache_get_stats(cache_manager_t* cache, cache_stats_t* stats) {
    if (!cache || !stats) return;
    
    memcpy(stats, &cache->stats, sizeof(cache_stats_t));
    
    uint64_t total_reads = stats->read_hits + stats->read_misses;
    uint64_t total_writes = stats->write_hits + stats->write_misses;
    
    stats->read_hit_rate = total_reads > 0 ? 
        (double)stats->read_hits / total_reads : 0.0;
    stats->write_hit_rate = total_writes > 0 ? 
        (double)stats->write_hits / total_writes : 0.0;
}

void cache_dump_info(cache_manager_t* cache) {
    if (!cache) return;
    
    cache_stats_t stats;
    cache_get_stats(cache, &stats);
    
    printf("\n[Cache] Manager Info:\n");
    printf("  Capacity: %u pages\n", cache->capacity);
    printf("  Used: %u pages (%.1f%%)\n", cache->used, 
           100.0 * cache->used / cache->capacity);
    printf("  Page size: %u KB\n", cache->page_size / 1024);
    printf("  Read hit rate: %.2f%%\n", stats.read_hit_rate * 100);
    printf("  Write hit rate: %.2f%%\n", stats.write_hit_rate * 100);
    printf("  Flushes: %lu\n", stats.flushes);
    printf("  Evictions: %lu\n", stats.evictions);
}

/* 辅助函数 */
static uint32_t hash_lba(uint64_t lba, uint32_t hash_size) {
    return (uint32_t)(lba % hash_size);
}

static cache_entry_t* cache_lookup(cache_manager_t* cache, uint64_t lba) {
    uint32_t hash = hash_lba(lba, cache->hash_size);
    cache_entry_t* entry = cache->hash_table[hash];
    
    while (entry) {
        if (entry->lba == lba && entry->state != CACHE_ENTRY_FREE) {
            return entry;
        }
        entry = entry->hash_next;
    }
    
    return NULL;
}

static cache_entry_t* cache_alloc_entry(cache_manager_t* cache) {
    /* 查找空闲条目 */
    for (uint32_t i = 0; i < cache->capacity; i++) {
        if (cache->entries[i].state == CACHE_ENTRY_FREE) {
            return &cache->entries[i];
        }
    }
    
    /* LRU淘汰 */
    cache_entry_t* victim = cache->lru_tail;
    if (!victim) return NULL;
    
    /* 如果脏，刷新 */
    if (victim->state == CACHE_ENTRY_DIRTY) {
        /* 实际实现会写入NAND */
        __sync_fetch_and_add(&cache->stats.flushes, 1);
    }
    
    __sync_fetch_and_add(&cache->stats.evictions, 1);
    
    /* 从Hash表移除 */
    uint32_t hash = hash_lba(victim->lba, cache->hash_size);
    cache_entry_t** pp = &cache->hash_table[hash];
    while (*pp) {
        if (*pp == victim) {
            *pp = victim->hash_next;
            break;
        }
        pp = &(*pp)->hash_next;
    }
    
    /* 从LRU链表移除 */
    cache_remove_lru(cache, victim);
    
    victim->hash_next = NULL;
    cache->used--;
    
    return victim;
}

static void cache_update_lru(cache_manager_t* cache, cache_entry_t* entry) {
    /* 从原位置移除 */
    cache_remove_lru(cache, entry);
    
    /* 插入头部 */
    entry->lru_next = cache->lru_head;
    entry->lru_prev = NULL;
    if (cache->lru_head) {
        cache->lru_head->lru_prev = entry;
    }
    cache->lru_head = entry;
    
    if (!cache->lru_tail) {
        cache->lru_tail = entry;
    }
}

static void cache_remove_lru(cache_manager_t* cache, cache_entry_t* entry) {
    if (entry->lru_prev) {
        entry->lru_prev->lru_next = entry->lru_next;
    } else {
        cache->lru_head = entry->lru_next;
    }
    
    if (entry->lru_next) {
        entry->lru_next->lru_prev = entry->lru_prev;
    } else {
        cache->lru_tail = entry->lru_prev;
    }
}

static inline uint64_t get_time_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}
