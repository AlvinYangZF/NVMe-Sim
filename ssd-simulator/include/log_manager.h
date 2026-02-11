/**
 * @file log_manager.h
 * @brief SSD Simulator - Log Manager Header
 * 
 * 提供分级日志记录功能，支持文件输出、控制台输出、日志轮转
 */

#ifndef __LOG_MANAGER_H__
#define __LOG_MANAGER_H__

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 日志级别 */
typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_FATAL,
    LOG_LEVEL_NUM
} log_level_t;

/* 日志输出目标 */
typedef enum {
    LOG_OUTPUT_NONE = 0,
    LOG_OUTPUT_CONSOLE = 1,
    LOG_OUTPUT_FILE = 2,
    LOG_OUTPUT_BOTH = 3
} log_output_t;

/* 日志配置 */
typedef struct log_config {
    log_level_t level;          /* 最低日志级别 */
    log_output_t output;        /* 输出目标 */
    const char* log_dir;        /* 日志目录 */
    const char* log_name;       /* 日志文件名 */
    uint32_t max_file_size;     /* 单个文件最大大小（MB） */
    uint32_t max_files;         /* 最大保留文件数 */
    bool enable_timestamp;      /* 启用时间戳 */
    bool enable_location;       /* 启用位置信息（文件:行号） */
    bool enable_thread_id;      /* 启用线程ID */
} log_config_t;

/* 日志统计 */
typedef struct log_stats {
    uint64_t debug_count;
    uint64_t info_count;
    uint64_t warn_count;
    uint64_t error_count;
    uint64_t fatal_count;
    uint64_t total_count;
    uint64_t dropped_count;
    uint64_t bytes_written;
} log_stats_t;

/* Opaque类型 */
typedef struct log_manager log_manager_t;

/**
 * @brief 获取默认配置
 * @return 默认配置
 */
log_config_t log_get_default_config(void);

/**
 * @brief 初始化日志管理器
 * @param config 日志配置（NULL使用默认配置）
 * @return 0成功，负数错误码
 */
int log_init(const log_config_t* config);

/**
 * @brief 销毁日志管理器
 */
void log_destroy(void);

/**
 * @brief 设置日志级别
 * @param level 日志级别
 */
void log_set_level(log_level_t level);

/**
 * @brief 获取当前日志级别
 * @return 当前日志级别
 */
log_level_t log_get_level(void);

/**
 * @brief 基础日志函数
 * @param level 日志级别
 * @param file 源文件名
 * @param line 行号
 * @param func 函数名
 * @param fmt 格式字符串
 * @param ... 可变参数
 */
void log_write(log_level_t level, const char* file, int line, 
               const char* func, const char* fmt, ...);

/**
 * @brief 获取日志统计
 * @param stats 统计输出
 */
void log_get_stats(log_stats_t* stats);

/**
 * @brief 刷新日志缓冲区
 */
void log_flush(void);

/**
 * @brief 获取日志级别字符串
 * @param level 日志级别
 * @return 级别字符串
 */
const char* log_level_to_string(log_level_t level);

/* 便捷宏 */
#define LOG_DEBUG(fmt, ...) \
    log_write(LOG_LEVEL_DEBUG, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

#define LOG_INFO(fmt, ...) \
    log_write(LOG_LEVEL_INFO, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

#define LOG_WARN(fmt, ...) \
    log_write(LOG_LEVEL_WARN, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

#define LOG_ERROR(fmt, ...) \
    log_write(LOG_LEVEL_ERROR, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

#define LOG_FATAL(fmt, ...) \
    log_write(LOG_LEVEL_FATAL, __FILE__, __LINE__, __func__, fmt, ##__VA_ARGS__)

/* 条件日志 */
#define LOG_DEBUG_IF(cond, fmt, ...) \
    do { if (cond) LOG_DEBUG(fmt, ##__VA_ARGS__); } while(0)

#define LOG_INFO_IF(cond, fmt, ...) \
    do { if (cond) LOG_INFO(fmt, ##__VA_ARGS__); } while(0)

#define LOG_WARN_IF(cond, fmt, ...) \
    do { if (cond) LOG_WARN(fmt, ##__VA_ARGS__); } while(0)

#define LOG_ERROR_IF(cond, fmt, ...) \
    do { if (cond) LOG_ERROR(fmt, ##__VA_ARGS__); } while(0)

#ifdef __cplusplus
}
#endif

#endif /* __LOG_MANAGER_H__ */
