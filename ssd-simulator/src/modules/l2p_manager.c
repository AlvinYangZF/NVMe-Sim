/**
 * @file l2p_manager.c
 * @brief SSD Simulator - L2P Manager Implementation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>
#include "l2p_manager.h"
#include "ddr_manager.h"

/* 每页L2P条目数 (16KB页 / 8字节条目) */
#define L2P_ENTRIES_PER_PAGE    2048

/* 缓存行大小（连续LBA数量） */
#define L2P_CACHE_LINE_SIZE     64

/* 缓存行 */
typedef struct l2p_cache_line {
    uint64_t start_lba;         /* 起始LBA */
    l2p_entry_t entries[L2P_CACHE_LINE_SIZE];  /* 64个连续LBA的映射 */
    uint64_t last_access;       /* 最后访问时间 */
    uint32_t access_count;      /* 访问计数 */
    uint8_t dirty;              /* 脏标志 */
    uint8_t valid;              /* 有效标志 */
    
    /* LRU链表 */
    struct l2p_cache_line* lru_next;
    struct l2p_cache_line* lru_prev;
    
    /* Hash链表 */
    struct l2p_cache_line* hash_next;
} l2p_cache_line_t;

/* L2P缓存 */
typedef struct l2p_cache {
    l2p_cache_line_t* lines;    /* 缓存行数组 */
    uint32_t num_lines;         /* 总行数 */
    uint32_t used_lines;        /* 已用行数 */
    
    /* Hash表 */
    l2p_cache_line_t** hash_table;
    uint32_t hash_size;
    
    /* LRU链表 */
    l2p_cache_line_t* lru_head;
    l2p_cache_line_t* lru_tail;
    
    pthread_rwlock_t lock;
} l2p_cache_t;

/* PPA分配器（简化实现） */
typedef struct ppa_allocator {
    uint64_t next_ppa;          /* 下一个分配位置 */
    uint64_t total_ppas;        /* 总PPA数 */
    pthread_spinlock_t lock;
} ppa_allocator_t;

/* L2P管理器 */
struct l2p_manager {
    uint64_t num_lbas;          /* 总LBA数 */
    
    /* 缓存 */
    l2p_cache_t cache;
    
    /* PPA分配器 */
    ppa_allocator_t ppa_alloc;
    
    /* 统计 */
    l2p_stats_t stats;
};

/* 前向声明 */
static uint32_t hash_lba(uint64_t lba, uint32_t hash_size);
static l2p_cache_line_t* cache_lookup(l2p_cache_t* cache, uint64_t lba);
static int cache_insert(l2p_cache_t* cache, uint64_t lba, l2p_entry_t* entries);
static l2p_cache_line_t* cache_alloc_line(l2p_cache_t* cache);
static void cache_update_lru(l2p_cache_t* cache, l2p_cache_line_t* line);
static void cache_remove_lru(l2p_cache_t* cache, l2p_cache_line_t* line);
static inline uint64_t get_time_ns(void);

l2p_manager_t* l2p_manager_create(uint64_t num_lbas, uint32_t cache_size) {
    if (num_lbas == 0 || cache_size == 0) return NULL;
    
    l2p_manager_t* l2p = calloc(1, sizeof(l2p_manager_t));
    if (!l2p) return NULL;
    
    l2p->num_lbas = num_lbas;
    
    /* 初始化缓存 */
    l2p->cache.num_lines = cache_size;
    l2p->cache.used_lines = 0;
    l2p->cache.lines = calloc(cache_size, sizeof(l2p_cache_line_t));
    if (!l2p->cache.lines) {
        free(l2p);
        return NULL;
    }
    
    /* 初始化Hash表 */
    l2p->cache.hash_size = cache_size * 2;
    l2p->cache.hash_table = calloc(l2p->cache.hash_size, sizeof(l2p_cache_line_t*));
    if (!l2p->cache.hash_table) {
        free(l2p->cache.lines);
        free(l2p);
        return NULL;
    }
    
    pthread_rwlock_init(&l2p->cache.lock, NULL);
    
    /* 初始化PPA分配器（简化：顺序分配） */
    pthread_spin_init(&l2p->ppa_alloc.lock, PTHREAD_PROCESS_PRIVATE);
    l2p->ppa_alloc.next_ppa = 0;
    l2p->ppa_alloc.total_ppas = num_lbas * 2;  /* 假设OP空间 */
    
    printf("[L2P] Manager created: %lu LBAs, cache %u lines\n", 
           num_lbas, cache_size);
    
    return l2p;
}

void l2p_manager_destroy(l2p_manager_t* l2p) {
    if (!l2p) return;
    
    /* 刷新脏缓存 */
    l2p_flush(l2p);
    
    pthread_rwlock_destroy(&l2p->cache.lock);
    pthread_spin_destroy(&l2p->ppa_alloc.lock);
    
    free(l2p->cache.hash_table);
    free(l2p->cache.lines);
    free(l2p);
    
    printf("[L2P] Manager destroyed\n");
}

int l2p_lookup(l2p_manager_t* l2p, uint64_t lba, l2p_entry_t* entry) {
    if (!l2p || !entry || lba >= l2p->num_lbas) return -EINVAL;
    
    l2p->stats.total_lookups++;
    
    /* 1. 查询缓存 */
    pthread_rwlock_rdlock(&l2p->cache.lock);
    l2p_cache_line_t* line = cache_lookup(&l2p->cache, lba);
    
    if (line && line->valid) {
        uint32_t offset = lba % L2P_CACHE_LINE_SIZE;
        *entry = line->entries[offset];
        
        line->last_access = get_time_ns();
        __sync_fetch_and_add(&line->access_count, 1);
        cache_update_lru(&l2p->cache, line);
        
        pthread_rwlock_unlock(&l2p->cache.lock);
        
        l2p->stats.cache_hits++;
        return (entry->flags & L2P_FLAG_VALID) ? 0 : -ENOENT;
    }
    
    pthread_rwlock_unlock(&l2p->cache.lock);
    
    /* 2. 缓存未命中，返回无效映射 */
    entry->ppa = L2P_INVALID_PPA;
    entry->flags = 0;
    
    l2p->stats.cache_misses++;
    return -ENOENT;
}

uint32_t l2p_lookup_batch(l2p_manager_t* l2p, uint64_t* lbas, 
                          l2p_entry_t* entries, uint32_t count) {
    if (!l2p || !lbas || !entries || count == 0) return 0;
    
    uint32_t found = 0;
    
    for (uint32_t i = 0; i < count; i++) {
        if (l2p_lookup(l2p, lbas[i], &entries[i]) == 0) {
            found++;
        }
    }
    
    return found;
}

int l2p_update(l2p_manager_t* l2p, uint64_t lba, uint64_t ppa, uint16_t flags) {
    if (!l2p || lba >= l2p->num_lbas) return -EINVAL;
    
    l2p->stats.total_updates++;
    
    pthread_rwlock_wrlock(&l2p->cache.lock);
    
    /* 查找缓存行 */
    l2p_cache_line_t* line = cache_lookup(&l2p->cache, lba);
    
    if (!line) {
        /* 分配新行 */
        line = cache_alloc_line(&l2p->cache);
        if (!line) {
            pthread_rwlock_unlock(&l2p->cache.lock);
            return -ENOMEM;
        }
        
        /* 初始化行 */
        line->start_lba = (lba / L2P_CACHE_LINE_SIZE) * L2P_CACHE_LINE_SIZE;
        line->valid = 1;
        line->dirty = 0;
        
        /* 加入Hash表 */
        uint32_t hash = hash_lba(line->start_lba, l2p->cache.hash_size);
        line->hash_next = l2p->cache.hash_table[hash];
        l2p->cache.hash_table[hash] = line;
        
        /* 初始化条目 */
        for (int i = 0; i < L2P_CACHE_LINE_SIZE; i++) {
            line->entries[i].ppa = L2P_INVALID_PPA;
            line->entries[i].flags = 0;
        }
        
        l2p->cache.used_lines++;
    }
    
    /* 更新条目 */
    uint32_t offset = lba % L2P_CACHE_LINE_SIZE;
    line->entries[offset].ppa = ppa;
    line->entries[offset].flags = flags | L2P_FLAG_VALID;
    line->dirty = 1;
    line->last_access = get_time_ns();
    
    cache_update_lru(&l2p->cache, line);
    
    pthread_rwlock_unlock(&l2p->cache.lock);
    
    return 0;
}

uint32_t l2p_update_batch(l2p_manager_t* l2p, uint64_t* lbas, uint64_t* ppas,
                          uint16_t* flags, uint32_t count) {
    if (!l2p || !lbas || !ppas || count == 0) return 0;
    
    uint32_t updated = 0;
    
    for (uint32_t i = 0; i < count; i++) {
        uint16_t f = flags ? flags[i] : L2P_FLAG_VALID;
        if (l2p_update(l2p, lbas[i], ppas[i], f) == 0) {
            updated++;
        }
    }
    
    return updated;
}

int l2p_invalidate(l2p_manager_t* l2p, uint64_t lba) {
    if (!l2p || lba >= l2p->num_lbas) return -EINVAL;
    
    pthread_rwlock_wrlock(&l2p->cache.lock);
    
    l2p_cache_line_t* line = cache_lookup(&l2p->cache, lba);
    if (line && line->valid) {
        uint32_t offset = lba % L2P_CACHE_LINE_SIZE;
        line->entries[offset].flags &= ~L2P_FLAG_VALID;
        line->entries[offset].flags |= L2P_FLAG_INVALID;
        line->dirty = 1;
    }
    
    pthread_rwlock_unlock(&l2p->cache.lock);
    
    return 0;
}

int l2p_flush(l2p_manager_t* l2p) {
    if (!l2p) return -EINVAL;
    
    printf("[L2P] Flushing cache...\n");
    
    pthread_rwlock_wrlock(&l2p->cache.lock);
    
    uint32_t flushed = 0;
    for (uint32_t i = 0; i < l2p->cache.num_lines; i++) {
        l2p_cache_line_t* line = &l2p->cache.lines[i];
        if (line->valid && line->dirty) {
            /* 实际实现中这里会写入NAND */
            line->dirty = 0;
            flushed++;
        }
    }
    
    pthread_rwlock_unlock(&l2p->cache.lock);
    
    printf("[L2P] Flushed %u lines\n", flushed);
    return 0;
}

int l2p_alloc_ppa(l2p_manager_t* l2p, uint64_t* ppa) {
    if (!l2p || !ppa) return -EINVAL;
    
    pthread_spin_lock(&l2p->ppa_alloc.lock);
    
    if (l2p->ppa_alloc.next_ppa >= l2p->ppa_alloc.total_ppas) {
        pthread_spin_unlock(&l2p->ppa_alloc.lock);
        return -ENOMEM;  /* 无空闲PPA */
    }
    
    *ppa = l2p->ppa_alloc.next_ppa++;
    
    pthread_spin_unlock(&l2p->ppa_alloc.lock);
    
    return 0;
}

int l2p_free_ppa(l2p_manager_t* l2p, uint64_t ppa) {
    /* 简化实现：暂不回收PPA */
    return 0;
}

void l2p_get_stats(l2p_manager_t* l2p, l2p_stats_t* stats) {
    if (!l2p || !stats) return;
    
    memcpy(stats, &l2p->stats, sizeof(l2p_stats_t));
    
    stats->cache_entries = l2p->cache.used_lines;
    
    if (stats->total_lookups > 0) {
        stats->hit_rate = (double)stats->cache_hits / stats->total_lookups;
    } else {
        stats->hit_rate = 0.0;
    }
}

void l2p_dump_info(l2p_manager_t* l2p) {
    if (!l2p) return;
    
    l2p_stats_t stats;
    l2p_get_stats(l2p, &stats);
    
    printf("\n[L2P] Manager Info:\n");
    printf("  Total LBAs: %lu\n", l2p->num_lbas);
    printf("  Cache lines: %u / %u\n", l2p->cache.used_lines, l2p->cache.num_lines);
    printf("  Lookups: %lu\n", stats.total_lookups);
    printf("  Cache hits: %lu (%.2f%%)\n", stats.cache_hits, stats.hit_rate * 100);
    printf("  Updates: %lu\n", stats.total_updates);
}

/* 辅助函数 */
static uint32_t hash_lba(uint64_t lba, uint32_t hash_size) {
    /* 简单的哈希函数 */
    return (uint32_t)((lba / L2P_CACHE_LINE_SIZE) % hash_size);
}

static l2p_cache_line_t* cache_lookup(l2p_cache_t* cache, uint64_t lba) {
    uint64_t start_lba = (lba / L2P_CACHE_LINE_SIZE) * L2P_CACHE_LINE_SIZE;
    uint32_t hash = hash_lba(start_lba, cache->hash_size);
    
    l2p_cache_line_t* line = cache->hash_table[hash];
    while (line) {
        if (line->valid && line->start_lba == start_lba) {
            return line;
        }
        line = line->hash_next;
    }
    
    return NULL;
}

static l2p_cache_line_t* cache_alloc_line(l2p_cache_t* cache) {
    /* 如果有空闲行 */
    if (cache->used_lines < cache->num_lines) {
        return &cache->lines[cache->used_lines];
    }
    
    /* LRU淘汰 */
    l2p_cache_line_t* victim = cache->lru_tail;
    if (!victim) return NULL;
    
    /* 从Hash表移除 */
    uint32_t hash = hash_lba(victim->start_lba, cache->hash_size);
    l2p_cache_line_t** pp = &cache->hash_table[hash];
    while (*pp) {
        if (*pp == victim) {
            *pp = victim->hash_next;
            break;
        }
        pp = &(*pp)->hash_next;
    }
    
    /* 从LRU链表移除 */
    cache_remove_lru(cache, victim);
    
    /* 如果脏，需要刷盘（简化：这里直接丢弃） */
    victim->hash_next = NULL;
    victim->lru_next = NULL;
    victim->lru_prev = NULL;
    
    return victim;
}

static void cache_update_lru(l2p_cache_t* cache, l2p_cache_line_t* line) {
    /* 从原位置移除 */
    cache_remove_lru(cache, line);
    
    /* 插入头部 */
    line->lru_next = cache->lru_head;
    line->lru_prev = NULL;
    if (cache->lru_head) {
        cache->lru_head->lru_prev = line;
    }
    cache->lru_head = line;
    
    if (!cache->lru_tail) {
        cache->lru_tail = line;
    }
}

static void cache_remove_lru(l2p_cache_t* cache, l2p_cache_line_t* line) {
    if (line->lru_prev) {
        line->lru_prev->lru_next = line->lru_next;
    } else {
        cache->lru_head = line->lru_next;
    }
    
    if (line->lru_next) {
        line->lru_next->lru_prev = line->lru_prev;
    } else {
        cache->lru_tail = line->lru_prev;
    }
}

static inline uint64_t get_time_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}
