#include "BenchmarkUtils.h"
#include <filesystem>

namespace fs = std::filesystem;

int main() {
    try {
        logger::log_info("start Key Interval A* benchmark");
        
        // auto solver = std::make_unique<KeyIntervalAStar>();
        auto solver = std::make_unique<HPAStar>();
        // auto solver = std::make_unique<AStar>();
        
        // run all scenarios
        BenchmarkUtils::run_all_scenarios_test(solver.get());
        
        logger::log_info("benchmark finished");
        
    } catch (const std::exception& e) {
        logger::log_error("Exception in main: " + std::string(e.what()));
        return 1;
    }
    
    return 0;
} 