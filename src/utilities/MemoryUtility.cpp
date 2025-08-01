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
        return pmc.WorkingSetSize;  // return working set size (physical memory)
    }
    return 0;
#else
    // Linux implementation - read /proc/self/status file
    FILE* file = fopen("/proc/self/status", "r");
    if (file) {
        char line[128];
        while (fgets(line, 128, file) != NULL) {
            if (strncmp(line, "VmRSS:", 6) == 0) {
                size_t memory;
                sscanf(line, "VmRSS: %zu", &memory);
                fclose(file);
                return memory * 1024; // convert to bytes
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
        // if memory decreased, return 0 (maybe memory recycling)
        return 0;
    }
}

double calculate_memory_increase_mb(size_t before, size_t after) {
    return static_cast<double>(calculate_memory_increase(before, after)) / (1024 * 1024);
}

} // namespace memory_utils 