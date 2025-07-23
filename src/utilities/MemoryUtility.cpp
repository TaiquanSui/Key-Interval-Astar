#include "MemoryUtility.h"

#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#else
#include <cstdio>
#include <cstring>
#include <unistd.h>
#endif

namespace memory_utils {

size_t get_current_memory_usage() {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS_EX pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc))) {
        return pmc.WorkingSetSize;  // 返回工作集大小（物理内存）
    }
    return 0;
#else
    // Linux实现 - 读取 /proc/self/status 文件
    FILE* file = fopen("/proc/self/status", "r");
    if (file) {
        char line[128];
        while (fgets(line, 128, file) != NULL) {
            if (strncmp(line, "VmRSS:", 6) == 0) {
                size_t memory;
                sscanf(line, "VmRSS: %zu", &memory);
                fclose(file);
                return memory * 1024; // 转换为字节
            }
        }
        fclose(file);
    }
    return 0;
#endif
}

double get_current_memory_usage_mb() {
    return static_cast<double>(get_current_memory_usage()) / (1024 * 1024);
}

size_t calculate_memory_increase(size_t before, size_t after) {
    if (after >= before) {
        return after - before;
    } else {
        // 如果内存减少了，返回0（可能是内存回收）
        return 0;
    }
}

double calculate_memory_increase_mb(size_t before, size_t after) {
    return static_cast<double>(calculate_memory_increase(before, after)) / (1024 * 1024);
}

} // namespace memory_utils 