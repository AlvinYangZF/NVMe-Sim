/**
 * @file nand_simulator.c
 * @brief SSD Simulator - NAND Simulator Implementation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include "nand_simulator.h"

/* NAND类型默认配置 */
static const nand_config_t g_nand_configs[NAND_TYPE_NUM] = {
    [NAND_TYPE_SLC] = {
        .type = NAND_TYPE_SLC,
        .pages_per_block = 64,
        .page_size = 16384,
        .oob_size = 1664,
        .timing = { 25, 200, 1500, 100 },
        .pe_cycles = 100000,
    },
    [NAND_TYPE_MLC] = {
        .type = NAND_TYPE_MLC,
        .pages_per_block = 256,
        .page_size = 16384,
        .oob_size = 1664,
        .timing = { 50, 400, 2000, 100 },
        .pe_cycles = 10000,
    },
    [NAND_TYPE_TLC] = {
        .type = NAND_TYPE_TLC,
        .pages_per_block = 384,
        .page_size = 16384,
        .oob_size = 1664,
        .timing = { 60, 600, 3000, 100 },
        .pe_cycles = 3000,
    },
    [NAND_TYPE_QLC] = {
        .type = NAND_TYPE_QLC,
        .pages_per_block = 512,
        .page_size = 16384,
        .oob_size = 1664,
        .timing = { 80, 800, 4000, 100 },
        .pe_cycles = 1000,
    },
};

/* NAND页 */
typedef struct nand_page {
    uint8_t* data;
    uint8_t* oob;
    page_state_t state;
    uint64_t written_time;
} nand_page_t;

/* NAND块 */
typedef struct nand_block {
    nand_page_t* pages;
    uint32_t num_pages;
    block_state_t state;
    uint32_t erase_count;
    uint32_t valid_pages;
    uint32_t invalid_pages;
    uint32_t next_page;     /* 顺序写入位置 */
} nand_block_t;

/* NAND Plane */
typedef struct nand_plane {
    nand_block_t* blocks;
    uint32_t num_blocks;
    uint32_t open_block;    /* 当前开放块 */
} nand_plane_t;

/* NAND Die */
typedef struct nand_die {
    nand_plane_t* planes;
    uint32_t num_planes;
} nand_die_t;

/* NAND Channel */
typedef struct nand_channel {
    nand_die_t* dies;
    uint32_t num_dies;
} nand_channel_t;

/* NAND模拟器 */
struct nand_simulator {
    nand_config_t config;
    nand_channel_t* channels;
    
    /* 错误注入 */
    double error_rate;
    
    /* 统计 */
    nand_stats_t stats;
    
    /* 总容量 */
    uint64_t total_pages;
    uint64_t total_blocks;
    uint64_t total_capacity;
};

/* 前向声明 */
static nand_page_t* get_page(nand_simulator_t* sim, uint64_t ppa);
static nand_block_t* get_block(nand_simulator_t* sim, uint64_t ppa);
static inline uint64_t get_time_us(void);

nand_simulator_t* nand_sim_create(nand_config_t* config) {
    if (!config) return NULL;
    
    nand_simulator_t* sim = calloc(1, sizeof(nand_simulator_t));
    if (!sim) return NULL;
    
    /* 复制配置 */
    memcpy(&sim->config, config, sizeof(nand_config_t));
    
    /* 应用类型默认参数 */
    const nand_config_t* type_config = &g_nand_configs[config->type];
    if (sim->config.pages_per_block == 0) {
        sim->config.pages_per_block = type_config->pages_per_block;
    }
    if (sim->config.page_size == 0) {
        sim->config.page_size = type_config->page_size;
    }
    if (sim->config.oob_size == 0) {
        sim->config.oob_size = type_config->oob_size;
    }
    if (sim->config.timing.t_read == 0) {
        sim->config.timing = type_config->timing;
    }
    if (sim->config.pe_cycles == 0) {
        sim->config.pe_cycles = type_config->pe_cycles;
    }
    
    /* 创建Channel */
    sim->channels = calloc(sim->config.num_channels, sizeof(nand_channel_t));
    if (!sim->channels) goto err_free_sim;
    
    for (uint32_t ch = 0; ch < sim->config.num_channels; ch++) {
        nand_channel_t* channel = &sim->channels[ch];
        channel->num_dies = sim->config.dies_per_channel;
        channel->dies = calloc(channel->num_dies, sizeof(nand_die_t));
        if (!channel->dies) goto err_cleanup;
        
        for (uint32_t die = 0; die < channel->num_dies; die++) {
            nand_die_t* d = &channel->dies[die];
            d->num_planes = sim->config.planes_per_die;
            d->planes = calloc(d->num_planes, sizeof(nand_plane_t));
            if (!d->planes) goto err_cleanup;
            
            for (uint32_t pl = 0; pl < d->num_planes; pl++) {
                nand_plane_t* plane = &d->planes[pl];
                plane->num_blocks = sim->config.blocks_per_plane;
                plane->blocks = calloc(plane->num_blocks, sizeof(nand_block_t));
                if (!plane->blocks) goto err_cleanup;
                
                plane->open_block = UINT32_MAX;
                
                for (uint32_t blk = 0; blk < plane->num_blocks; blk++) {
                    nand_block_t* block = &plane->blocks[blk];
                    block->num_pages = sim->config.pages_per_block;
                    block->pages = calloc(block->num_pages, sizeof(nand_page_t));
                    if (!block->pages) goto err_cleanup;
                    
                    block->state = BLOCK_STATE_FREE;
                    block->next_page = 0;
                    
                    for (uint32_t pg = 0; pg < block->num_pages; pg++) {
                        nand_page_t* page = &block->pages[pg];
                        page->data = calloc(1, sim->config.page_size);
                        page->oob = calloc(1, sim->config.oob_size);
                        page->state = PAGE_STATE_FREE;
                        
                        /* 初始化为0xFF */
                        memset(page->data, 0xFF, sim->config.page_size);
                        memset(page->oob, 0xFF, sim->config.oob_size);
                    }
                }
            }
        }
    }
    
    /* 计算总容量 */
    sim->total_blocks = (uint64_t)sim->config.num_channels * 
                        sim->config.dies_per_channel * 
                        sim->config.planes_per_die * 
                        sim->config.blocks_per_plane;
    sim->total_pages = sim->total_blocks * sim->config.pages_per_block;
    sim->total_capacity = sim->total_pages * sim->config.page_size;
    
    printf("[NAND] Simulator created: %lu GB\n", sim->total_capacity / (1024 * 1024 * 1024));
    printf("       Channels: %u, Dies/Ch: %u, Planes/Die: %u\n",
           sim->config.num_channels, sim->config.dies_per_channel, sim->config.planes_per_die);
    printf("       Blocks/Plane: %u, Pages/Block: %u, Page Size: %u KB\n",
           sim->config.blocks_per_plane, sim->config.pages_per_block, 
           sim->config.page_size / 1024);
    
    return sim;

err_cleanup:
    nand_sim_destroy(sim);
    return NULL;

err_free_sim:
    free(sim);
    return NULL;
}

void nand_sim_destroy(nand_simulator_t* sim) {
    if (!sim) return;
    
    for (uint32_t ch = 0; ch < sim->config.num_channels; ch++) {
        nand_channel_t* channel = &sim->channels[ch];
        if (!channel->dies) continue;
        
        for (uint32_t die = 0; die < channel->num_dies; die++) {
            nand_die_t* d = &channel->dies[die];
            if (!d->planes) continue;
            
            for (uint32_t pl = 0; pl < d->num_planes; pl++) {
                nand_plane_t* plane = &d->planes[pl];
                if (!plane->blocks) continue;
                
                for (uint32_t blk = 0; blk < plane->num_blocks; blk++) {
                    nand_block_t* block = &plane->blocks[blk];
                    if (!block->pages) continue;
                    
                    for (uint32_t pg = 0; pg < block->num_pages; pg++) {
                        nand_page_t* page = &block->pages[pg];
                        free(page->data);
                        free(page->oob);
                    }
                    free(block->pages);
                }
                free(plane->blocks);
            }
            free(d->planes);
        }
        free(channel->dies);
    }
    free(sim->channels);
    free(sim);
    
    printf("[NAND] Simulator destroyed\n");
}

int nand_read_page(nand_simulator_t* sim, uint64_t ppa, void* data_buf, void* oob_buf) {
    if (!sim || !data_buf) return -EINVAL;
    
    nand_page_t* page = get_page(sim, ppa);
    if (!page) return -EINVAL;
    
    /* 模拟读取延迟 */
    uint64_t start_time = get_time_us();
    uint32_t latency_us = sim->config.timing.t_read;
    
    /* 错误注入 */
    if (sim->error_rate > 0 && (rand() / (double)RAND_MAX) < sim->error_rate) {
        sim->stats.read_error_count++;
        /* 注入位翻转 */
        for (uint32_t i = 0; i < sim->config.page_size; i++) {
            if ((rand() / (double)RAND_MAX) < sim->error_rate / 10) {
                ((uint8_t*)page->data)[i] ^= (1 << (rand() % 8));
            }
        }
    }
    
    /* 复制数据 */
    memcpy(data_buf, page->data, sim->config.page_size);
    if (oob_buf) {
        memcpy(oob_buf, page->oob, sim->config.oob_size);
    }
    
    /* 统计 */
    sim->stats.read_count++;
    sim->stats.total_read_latency += latency_us;
    sim->stats.bytes_read += sim->config.page_size;
    
    return 0;
}

int nand_write_page(nand_simulator_t* sim, uint64_t ppa, void* data_buf, void* oob_buf) {
    if (!sim || !data_buf) return -EINVAL;
    
    nand_page_t* page = get_page(sim, ppa);
    if (!page) return -EINVAL;
    
    nand_block_t* block = get_block(sim, ppa);
    if (!block) return -EINVAL;
    
    /* 检查块状态 */
    if (block->state == BLOCK_STATE_BAD) {
        return -EIO;
    }
    
    /* 检查页状态 */
    if (page->state != PAGE_STATE_FREE) {
        return -EEXIST;  /* NAND不允许覆盖写 */
    }
    
    /* 必须按顺序写入 */
    uint32_t page_id = PPA_GET_PAGE(ppa);
    if (block->state == BLOCK_STATE_OPEN && page_id != block->next_page) {
        return -EINVAL;
    }
    
    /* 模拟编程延迟 */
    uint64_t start_time = get_time_us();
    uint32_t latency_us = sim->config.timing.t_prog;
    
    /* 错误注入 */
    if (sim->error_rate > 0 && (rand() / (double)RAND_MAX) < sim->error_rate / 2) {
        sim->stats.prog_error_count++;
        return -EIO;
    }
    
    /* 写入数据 */
    memcpy(page->data, data_buf, sim->config.page_size);
    if (oob_buf) {
        memcpy(page->oob, oob_buf, sim->config.oob_size);
    }
    
    /* 更新状态 */
    page->state = PAGE_STATE_VALID;
    page->written_time = start_time;
    
    block->valid_pages++;
    block->next_page = page_id + 1;
    
    if (block->state == BLOCK_STATE_FREE) {
        block->state = BLOCK_STATE_OPEN;
    }
    
    if (block->next_page >= block->num_pages) {
        block->state = BLOCK_STATE_FULL;
    }
    
    /* 统计 */
    sim->stats.prog_count++;
    sim->stats.total_prog_latency += latency_us;
    sim->stats.bytes_written += sim->config.page_size;
    
    return 0;
}

int nand_erase_block(nand_simulator_t* sim, uint64_t ppa) {
    if (!sim) return -EINVAL;
    
    nand_block_t* block = get_block(sim, ppa);
    if (!block) return -EINVAL;
    
    /* 模拟擦除延迟 */
    uint64_t start_time = get_time_us();
    uint32_t latency_us = sim->config.timing.t_erase;
    
    /* 错误注入 */
    if (sim->error_rate > 0 && (rand() / (double)RAND_MAX) < sim->error_rate / 5) {
        block->state = BLOCK_STATE_BAD;
        sim->stats.erase_error_count++;
        return -EIO;
    }
    
    /* 擦除所有页 */
    for (uint32_t pg = 0; pg < block->num_pages; pg++) {
        nand_page_t* page = &block->pages[pg];
        memset(page->data, 0xFF, sim->config.page_size);
        memset(page->oob, 0xFF, sim->config.oob_size);
        page->state = PAGE_STATE_FREE;
    }
    
    /* 更新块状态 */
    block->state = BLOCK_STATE_FREE;
    block->erase_count++;
    block->valid_pages = 0;
    block->invalid_pages = 0;
    block->next_page = 0;
    
    /* 统计 */
    sim->stats.erase_count++;
    sim->stats.total_erase_latency += latency_us;
    
    return 0;
}

page_state_t nand_get_page_state(nand_simulator_t* sim, uint64_t ppa) {
    nand_page_t* page = get_page(sim, ppa);
    return page ? page->state : PAGE_STATE_FREE;
}

block_state_t nand_get_block_state(nand_simulator_t* sim, uint64_t ppa) {
    nand_block_t* block = get_block(sim, ppa);
    return block ? block->state : BLOCK_STATE_BAD;
}

uint32_t nand_get_erase_count(nand_simulator_t* sim, uint64_t ppa) {
    nand_block_t* block = get_block(sim, ppa);
    return block ? block->erase_count : 0;
}

int nand_mark_bad_block(nand_simulator_t* sim, uint64_t ppa) {
    nand_block_t* block = get_block(sim, ppa);
    if (!block) return -EINVAL;
    
    block->state = BLOCK_STATE_BAD;
    return 0;
}

int nand_is_bad_block(nand_simulator_t* sim, uint64_t ppa) {
    return nand_get_block_state(sim, ppa) == BLOCK_STATE_BAD;
}

void nand_set_error_rate(nand_simulator_t* sim, double error_rate) {
    if (sim) {
        sim->error_rate = error_rate;
    }
}

void nand_get_stats(nand_simulator_t* sim, nand_stats_t* stats) {
    if (!sim || !stats) return;
    memcpy(stats, &sim->stats, sizeof(nand_stats_t));
}

void nand_dump_info(nand_simulator_t* sim) {
    if (!sim) return;
    
    printf("\n[NAND] Device Info:\n");
    printf("  Type: %s\n", sim->config.type == NAND_TYPE_SLC ? "SLC" :
                          sim->config.type == NAND_TYPE_MLC ? "MLC" :
                          sim->config.type == NAND_TYPE_TLC ? "TLC" : "QLC");
    printf("  Capacity: %lu GB\n", sim->total_capacity / (1024 * 1024 * 1024));
    printf("  Total Blocks: %lu\n", sim->total_blocks);
    printf("  Total Pages: %lu\n", sim->total_pages);
    printf("  Page Size: %u KB + %u B OOB\n", 
           sim->config.page_size / 1024, sim->config.oob_size);
    
    printf("\n  Timing (us):\n");
    printf("    Read:  %u\n", sim->config.timing.t_read);
    printf("    Prog:  %u\n", sim->config.timing.t_prog);
    printf("    Erase: %u\n", sim->config.timing.t_erase);
    
    printf("\n  Statistics:\n");
    printf("    Reads:  %lu, Errors: %lu\n", sim->stats.read_count, sim->stats.read_error_count);
    printf("    Progs:  %lu, Errors: %lu\n", sim->stats.prog_count, sim->stats.prog_error_count);
    printf("    Erases: %lu, Errors: %lu\n", sim->stats.erase_count, sim->stats.erase_error_count);
}

/* 辅助函数 */
static nand_page_t* get_page(nand_simulator_t* sim, uint64_t ppa) {
    uint32_t ch = PPA_GET_CHANNEL(ppa);
    uint32_t die = PPA_GET_DIE(ppa);
    uint32_t pl = PPA_GET_PLANE(ppa);
    uint32_t blk = PPA_GET_BLOCK(ppa);
    uint32_t pg = PPA_GET_PAGE(ppa);
    
    if (ch >= sim->config.num_channels) return NULL;
    if (die >= sim->config.dies_per_channel) return NULL;
    if (pl >= sim->config.planes_per_die) return NULL;
    if (blk >= sim->config.blocks_per_plane) return NULL;
    if (pg >= sim->config.pages_per_block) return NULL;
    
    return &sim->channels[ch].dies[die].planes[pl].blocks[blk].pages[pg];
}

static nand_block_t* get_block(nand_simulator_t* sim, uint64_t ppa) {
    uint32_t ch = PPA_GET_CHANNEL(ppa);
    uint32_t die = PPA_GET_DIE(ppa);
    uint32_t pl = PPA_GET_PLANE(ppa);
    uint32_t blk = PPA_GET_BLOCK(ppa);
    
    if (ch >= sim->config.num_channels) return NULL;
    if (die >= sim->config.dies_per_channel) return NULL;
    if (pl >= sim->config.planes_per_die) return NULL;
    if (blk >= sim->config.blocks_per_plane) return NULL;
    
    return &sim->channels[ch].dies[die].planes[pl].blocks[blk];
}

static inline uint64_t get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000;
}
