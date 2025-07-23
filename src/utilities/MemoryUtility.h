#ifndef MEMORY_UTILITY_H
#define MEMORY_UTILITY_H

#include <cstddef>

namespace memory_utils {

// 获取当前进程内存使用量（字节）
size_t get_current_memory_usage();

// 获取当前进程内存使用量（MB）
double get_current_memory_usage_mb();

// 计算内存增长量（字节）
size_t calculate_memory_increase(size_t before, size_t after);

// 计算内存增长量（MB）
double calculate_memory_increase_mb(size_t before, size_t after);

} // namespace memory_utils

#endif // MEMORY_UTILITY_H 