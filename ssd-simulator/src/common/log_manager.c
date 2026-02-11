/**
 * @file log_manager.c
 * @brief SSD Simulator - Log Manager Implementation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include "log_manager.h"

/* 默认配置 */
#define DEFAULT_LOG_LEVEL       LOG_LEVEL_INFO
#define DEFAULT_OUTPUT          LOG_OUTPUT_BOTH
#define DEFAULT_LOG_DIR         "./logs"
#define DEFAULT_LOG_NAME        "ssd_simulator"
#define DEFAULT_MAX_FILE_SIZE   100    /* 100MB */
#define DEFAULT_MAX_FILES       10

/* 缓冲区大小 */
#define LOG_BUF_SIZE            4096
#define LOG_MSG_SIZE            2048

/* 日志级别字符串 */
static const char* g_level_strings[] = {
    "DEBUG", "INFO", "WARN", "ERROR", "FATAL"
};

static const char* g_level_colors[] = {
    "\033[36m",  /* CYAN - DEBUG */
    "\033[32m",  /* GREEN - INFO */
    "\033[33m",  /* YELLOW - WARN */
    "\033[31m",  /* RED - ERROR */
    "\033[35m",  /* MAGENTA - FATAL */
};

static const char* g_color_reset = "\033[0m";

/* 日志管理器 */
struct log_manager {
    log_config_t config;
    FILE* fp;
    char current_file[256];
    uint32_t current_file_size;
    
    pthread_mutex_t lock;
    pthread_t flush_thread;
    volatile int running;
    
    /* 统计 */
    log_stats_t stats;
    pthread_mutex_t stats_lock;
    
    /* 缓冲区 */
    char buffer[LOG_BUF_SIZE];
    uint32_t buffer_used;
};

/* 全局实例 */
static log_manager_t g_log = {0};
static int g_initialized = 0;

/* 前向声明 */
static int log_open_file(void);
static void log_close_file(void);
static int log_rotate(void);
static void log_write_to_file(const char* msg, size_t len);
static void log_write_to_console(const char* msg, log_level_t level);
static void* log_flush_thread(void* arg);
static void log_get_timestamp(char* buf, size_t size);
static uint32_t log_get_file_size(const char* path);

log_config_t log_get_default_config(void) {
    log_config_t config = {
        .level = DEFAULT_LOG_LEVEL,
        .output = DEFAULT_OUTPUT,
        .log_dir = DEFAULT_LOG_DIR,
        .log_name = DEFAULT_LOG_NAME,
        .max_file_size = DEFAULT_MAX_FILE_SIZE,
        .max_files = DEFAULT_MAX_FILES,
        .enable_timestamp = true,
        .enable_location = true,
        .enable_thread_id = false
    };
    return config;
}

int log_init(const log_config_t* config) {
    if (g_initialized) {
        return 0;  /* 已初始化 */
    }
    
    memset(&g_log, 0, sizeof(g_log));
    
    /* 应用配置 */
    if (config) {
        memcpy(&g_log.config, config, sizeof(log_config_t));
    } else {
        g_log.config = log_get_default_config();
    }
    
    /* 创建日志目录 */
    struct stat st = {0};
    if (stat(g_log.config.log_dir, &st) == -1) {
        if (mkdir(g_log.config.log_dir, 0755) != 0 && errno != EEXIST) {
            fprintf(stderr, "[LOG] Failed to create log directory: %s\n", strerror(errno));
            return -1;
        }
    }
    
    /* 初始化锁 */
    pthread_mutex_init(&g_log.lock, NULL);
    pthread_mutex_init(&g_log.stats_lock, NULL);
    
    /* 打开日志文件 */
    if (g_log.config.output & LOG_OUTPUT_FILE) {
        if (log_open_file() != 0) {
            pthread_mutex_destroy(&g_log.lock);
            pthread_mutex_destroy(&g_log.stats_lock);
            return -1;
        }
    }
    
    /* 启动刷新线程 */
    g_log.running = 1;
    pthread_create(&g_log.flush_thread, NULL, log_flush_thread, NULL);
    
    g_initialized = 1;
    
    LOG_INFO("Log manager initialized (level=%s, output=%d)",
             log_level_to_string(g_log.config.level), g_log.config.output);
    
    return 0;
}

void log_destroy(void) {
    if (!g_initialized) return;
    
    LOG_INFO("Log manager shutting down...");
    
    /* 停止刷新线程 */
    g_log.running = 0;
    pthread_join(g_log.flush_thread, NULL);
    
    /* 刷新剩余缓冲区 */
    log_flush();
    
    /* 关闭文件 */
    log_close_file();
    
    /* 销毁锁 */
    pthread_mutex_destroy(&g_log.lock);
    pthread_mutex_destroy(&g_log.stats_lock);
    
    g_initialized = 0;
}

void log_set_level(log_level_t level) {
    if (!g_initialized) return;
    g_log.config.level = level;
}

log_level_t log_get_level(void) {
    if (!g_initialized) return LOG_LEVEL_INFO;
    return g_log.config.level;
}

void log_write(log_level_t level, const char* file, int line, 
               const char* func, const char* fmt, ...) {
    if (!g_initialized || level < g_log.config.level) {
        /* 更新丢弃统计 */
        if (g_initialized) {
            pthread_mutex_lock(&g_log.stats_lock);
            g_log.stats.dropped_count++;
            pthread_mutex_unlock(&g_log.stats_lock);
        }
        return;
    }
    
    char msg[LOG_MSG_SIZE];
    char* p = msg;
    size_t remaining = LOG_MSG_SIZE;
    
    /* 时间戳 */
    if (g_log.config.enable_timestamp) {
        char timestamp[32];
        log_get_timestamp(timestamp, sizeof(timestamp));
        int n = snprintf(p, remaining, "[%s] ", timestamp);
        p += n;
        remaining -= n;
    }
    
    /* 日志级别 */
    int n = snprintf(p, remaining, "[%s] ", log_level_to_string(level));
    p += n;
    remaining -= n;
    
    /* 线程ID */
    if (g_log.config.enable_thread_id) {
        n = snprintf(p, remaining, "[TID:%lu] ", pthread_self());
        p += n;
        remaining -= n;
    }
    
    /* 位置信息 */
    if (g_log.config.enable_location && level >= LOG_LEVEL_WARN) {
        const char* basename = strrchr(file, '/');
        if (!basename) basename = file;
        else basename++;
        n = snprintf(p, remaining, "[%s:%d %s] ", basename, line, func);
        p += n;
        remaining -= n;
    }
    
    /* 日志内容 */
    va_list args;
    va_start(args, fmt);
    vsnprintf(p, remaining, fmt, args);
    va_end(args);
    
    /* 添加换行 */
    strncat(msg, "\n", LOG_MSG_SIZE - strlen(msg) - 1);
    
    size_t msg_len = strlen(msg);
    
    /* 写入输出 */
    pthread_mutex_lock(&g_log.lock);
    
    if (g_log.config.output & LOG_OUTPUT_FILE) {
        log_write_to_file(msg, msg_len);
    }
    
    if (g_log.config.output & LOG_OUTPUT_CONSOLE) {
        log_write_to_console(msg, level);
    }
    
    pthread_mutex_unlock(&g_log.lock);
    
    /* 更新统计 */
    pthread_mutex_lock(&g_log.stats_lock);
    g_log.stats.total_count++;
    g_log.stats.bytes_written += msg_len;
    switch (level) {
        case LOG_LEVEL_DEBUG: g_log.stats.debug_count++; break;
        case LOG_LEVEL_INFO:  g_log.stats.info_count++; break;
        case LOG_LEVEL_WARN:  g_log.stats.warn_count++; break;
        case LOG_LEVEL_ERROR: g_log.stats.error_count++; break;
        case LOG_LEVEL_FATAL: g_log.stats.fatal_count++; break;
        default: break;
    }
    pthread_mutex_unlock(&g_log.stats_lock);
}

void log_get_stats(log_stats_t* stats) {
    if (!g_initialized || !stats) return;
    pthread_mutex_lock(&g_log.stats_lock);
    memcpy(stats, &g_log.stats, sizeof(log_stats_t));
    pthread_mutex_unlock(&g_log.stats_lock);
}

void log_flush(void) {
    if (!g_initialized) return;
    pthread_mutex_lock(&g_log.lock);
    if (g_log.fp) {
        fflush(g_log.fp);
    }
    pthread_mutex_unlock(&g_log.lock);
}

const char* log_level_to_string(log_level_t level) {
    if (level < 0 || level >= LOG_LEVEL_NUM) return "UNKNOWN";
    return g_level_strings[level];
}

/* 辅助函数 */
static int log_open_file(void) {
    time_t now = time(NULL);
    struct tm* tm_info = localtime(&now);
    
    snprintf(g_log.current_file, sizeof(g_log.current_file),
             "%s/%s_%04d%02d%02d_%02d%02d%02d.log",
             g_log.config.log_dir, g_log.config.log_name,
             tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
    
    g_log.fp = fopen(g_log.current_file, "a");
    if (!g_log.fp) {
        fprintf(stderr, "[LOG] Failed to open log file: %s\n", strerror(errno));
        return -1;
    }
    
    /* 设置行缓冲 */
    setvbuf(g_log.fp, NULL, _IOLBF, 0);
    
    g_log.current_file_size = log_get_file_size(g_log.current_file);
    
    return 0;
}

static void log_close_file(void) {
    if (g_log.fp) {
        fclose(g_log.fp);
        g_log.fp = NULL;
    }
}

static int log_rotate(void) {
    log_close_file();
    return log_open_file();
}

static void log_write_to_file(const char* msg, size_t len) {
    if (!g_log.fp) return;
    
    /* 检查是否需要轮转 */
    if (g_log.current_file_size + len > g_log.config.max_file_size * 1024 * 1024) {
        if (log_rotate() != 0) return;
    }
    
    fwrite(msg, 1, len, g_log.fp);
    g_log.current_file_size += len;
}

static void log_write_to_console(const char* msg, log_level_t level) {
    if (level < LOG_LEVEL_WARN) {
        printf("%s%s%s", g_level_colors[level], msg, g_color_reset);
    } else {
        fprintf(stderr, "%s%s%s", g_level_colors[level], msg, g_color_reset);
    }
}

static void* log_flush_thread(void* arg) {
    (void)arg;
    while (g_log.running) {
        sleep(1);  /* 每秒刷新一次 */
        log_flush();
    }
    return NULL;
}

static void log_get_timestamp(char* buf, size_t size) {
    struct timeval tv;
    struct tm* tm_info;
    
    gettimeofday(&tv, NULL);
    tm_info = localtime(&tv.tv_sec);
    
    snprintf(buf, size, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
             tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec,
             (int)(tv.tv_usec / 1000));
}

static uint32_t log_get_file_size(const char* path) {
    struct stat st;
    if (stat(path, &st) == 0) {
        return st.st_size;
    }
    return 0;
}
