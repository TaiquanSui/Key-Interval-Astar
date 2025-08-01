#ifndef MEMORY_UTILITY_H
#define MEMORY_UTILITY_H

#include <cstddef>

namespace memory_utils {

// get current process memory usage (bytes)
size_t get_current_memory_usage();

// get current process memory usage (MB)
double get_current_memory_usage_mb();

// calculate memory increase (bytes)
size_t calculate_memory_increase(size_t before, size_t after);

// calculate memory increase (MB)
double calculate_memory_increase_mb(size_t before, size_t after);

} // namespace memory_utils

#endif // MEMORY_UTILITY_H 