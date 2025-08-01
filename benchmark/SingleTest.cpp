#include "BenchmarkUtils.h"
#include <filesystem>

namespace fs = std::filesystem;

int main() {
    try {
        // specify map and scenario to test
        // std::string map_name = "maze-128-128-1";
        // std::string map_name = "Berlin_1_256";
        // std::string map_name = "random-64-64-10";
        std::string map_name = "room-64-64-16";

        std::string scenario_type = "even";  // even or "random"
        int scenario_number = 2;  // scenario number 1-25

        logger::log_info("Start testing Key Interval A* algorithm");
        logger::log_info("Map: " + map_name);
        logger::log_info("Scenario type: " + scenario_type);
        logger::log_info("Scenario number: " + std::to_string(scenario_number));
        
        // build file path
        std::string root_dir = BenchmarkUtils::get_project_root();
        fs::path data_dir = fs::path(root_dir) / "data";
        fs::path map_path = data_dir / "mapf-map" / (map_name + ".map");
        
        // check if map file exists
        if (!fs::exists(map_path)) {
            logger::log_error("Map file not found: " + map_path.string());
            return 1;
        }
        
        // load map and create solver
        auto grid = load_map(map_path.string());
        // auto solver = std::make_unique<KeyIntervalAStar>();
        auto solver = std::make_unique<HPAStar>();
        
        // run single scenario test
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
            117,
            solver.get()
        );

        logger::log_info("Testing completed");
        
    } catch (const std::exception& e) {
        logger::log_error("Exception in main: " + std::string(e.what()));
        return 1;
    }
    
    return 0;
} 