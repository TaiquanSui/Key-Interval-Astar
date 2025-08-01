#include "BenchmarkUtils.h"
#include <filesystem>

namespace fs = std::filesystem;

int main() {
    try {
        logger::log_info("start Key Interval A* benchmark");
        
        // 创建solver（不需要预先预处理）
        // auto solver = std::make_unique<KeyIntervalAStar>();
        auto solver = std::make_unique<HPAStar>();
        // auto solver = std::make_unique<AStar>();

        
        // 运行所有场景的测试（每个地图会自动进行预处理）
        BenchmarkUtils::run_all_scenarios_test(solver.get());
        
        logger::log_info("benchmark finished");
        
    } catch (const std::exception& e) {
        logger::log_error("Exception in main: " + std::string(e.what()));
        return 1;
    }
    
    return 0;
} 