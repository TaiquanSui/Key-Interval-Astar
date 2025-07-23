#include "BenchmarkUtils.h"
#include <filesystem>

namespace fs = std::filesystem;

int main() {
    try {
        // 指定要测试的地图和场景
        // std::string map_name = "maze-128-128-1";
        // std::string map_name = "Berlin_1_256";
        // std::string map_name = "random-64-64-10";
        std::string map_name = "room-64-64-16";

        std::string scenario_type = "even";  // even 或 "random"
        int scenario_number = 1;  // 场景编号 1-25

        logger::log_info("Start testing Key Interval A* algorithm");
        logger::log_info("Map: " + map_name);
        logger::log_info("Scenario type: " + scenario_type);
        logger::log_info("Scenario number: " + std::to_string(scenario_number));
        
        // 构建文件路径
        std::string root_dir = BenchmarkUtils::get_project_root();
        fs::path data_dir = fs::path(root_dir) / "data";
        fs::path map_path = data_dir / "mapf-map" / (map_name + ".map");
        
        // 检查地图文件是否存在
        if (!fs::exists(map_path)) {
            logger::log_error("Map file not found: " + map_path.string());
            return 1;
        }
        
        // 加载地图并创建solver
        auto grid = load_map(map_path.string());
        auto solver = std::make_unique<KeyIntervalAStar>();
        // auto solver = std::make_unique<HPAStar>();
        
        // 运行单个场景测试
        // BenchmarkUtils::run_single_scenario_test(
        //     map_name,
        //     scenario_type,
        //     scenario_number,
        //     solver.get()
        // );

        BenchmarkUtils::run_single_agent_test(
            map_name,
            scenario_type,
            scenario_number,
            335,
            solver.get()
        );
        
        logger::log_info("Testing completed");
        
    } catch (const std::exception& e) {
        logger::log_error("Exception in main: " + std::string(e.what()));
        return 1;
    }
    
    return 0;
} 