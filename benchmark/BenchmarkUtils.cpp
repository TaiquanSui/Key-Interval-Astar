#include "BenchmarkUtils.h"
#include <algorithm>
#include <future>
#include <thread>
#include <limits>
#include <windows.h>
#include <psapi.h>
#include <cstdio>
#include <cstring>

void SingleAgentStats::print() const {
    std::cout << std::fixed << std::setprecision(2)
              << "\nBenchmark Stats:\n"
              << "  Total Instances: " << total_instances << "\n"
              << "  Successful Instances: " << successful_instances << "\n"
              << "  Success Rate: " << (success_rate * 100) << "%\n"
              << "  Average Runtime: " << avg_runtime << "s\n"
              << "  Average Path Length: " << avg_path_length << "\n"
              << "  Average Nodes Expanded: " << avg_nodes_expanded << "\n"
              << "========================================" << std::endl;
}

std::vector<SingleAgentResult> BenchmarkUtils::run_scen_file_test(
    const std::string& map_file,
    const std::string& scen_file,
    SolverInterface* solver,
    double time_limit) {
    
    // 设置CPU亲和性到最空闲的核心
    setCpuAffinity();
    
    // 加载地图和场景
    auto grid = load_map(map_file);
    auto agents = load_scen(scen_file, grid);
    
    fs::path map_path(map_file);
    fs::path scen_path(scen_file);
    std::string map_name = map_path.filename().string();
    std::string scen_name = scen_path.filename().string();
    
    std::vector<SingleAgentResult> results;
    
    logger::log_info("Testing " + map_name + " with " + std::to_string(agents.size()) + " agents using " + solver->get_name());
    
    // 注意：预处理现在在solver构造函数中完成
    logger::log_info("Solver initialized and ready for testing");
    
    // 测试每个智能体
    for (size_t i = 0; i < agents.size(); ++i) {
        const auto& agent = agents[i];
        
        //logger::log_info("Testing agent " + std::to_string(i + 1) +
        //                " from (" + std::to_string(agent.start.x) + "," + std::to_string(agent.start.y) + ")" +
        //                " to (" + std::to_string(agent.goal.x) + "," + std::to_string(agent.goal.y) + ")");
        
        try {
            // 重置扩展节点计数和搜索时间
            solver->resetExpandedNodes();
            solver->resetSearchTime();
            
            // 使用异步执行以支持超时
            std::future<std::vector<Vertex>> future = 
                std::async(std::launch::async, 
                         [solver, &agent]() {
                             return solver->search(agent.start, agent.goal);
                         });
            
            bool timeout = false;
            std::vector<Vertex> path;
            
            if (future.wait_for(std::chrono::duration<double>(time_limit)) == 
                std::future_status::timeout) {
                timeout = true;
                logger::log_info("Solver timeout (" + std::to_string(time_limit) + " seconds)");
            } else {
                try {
                    path = future.get();
                } catch (const std::exception& e) {
                    logger::log_error("Solver error: " + std::string(e.what()));
                    timeout = true;
                    path.clear();
                }
            }
            
            // 使用算法内部的计时
            double duration = timeout ? time_limit : solver->getSearchTime();
            
            bool success = !timeout && !path.empty();
            double path_length = success ? path.size() - 1 : 0.0;
            
            // 获取扩展节点数量
            int nodes_expanded = success ? solver->getExpandedNodes() : -1;
            
            SingleAgentResult result = {
                success,
                duration,
                path_length,
                nodes_expanded,
                solver->getPreprocessMemoryUsage(),
                solver->getSearchMemoryIncrease(),
                map_name,
                scen_name,
                agent.start,
                agent.goal
            };
            
            results.push_back(result);
            //print_result(result);
            
        } catch (const std::exception& e) {
            logger::log_error("Error testing agent " + std::to_string(i + 1) + ": " + std::string(e.what()));
            
            SingleAgentResult result = {
                false,
                0,
                0,
                -1,
                solver->getPreprocessMemoryUsage(),
                0,  // search_memory_increase
                map_name,
                scen_name,
                agent.start,
                agent.goal
            };
            results.push_back(result);
        }
    }
    
    // 恢复线程优先级
    restore_thread_priority();
    
    return results;
}

void BenchmarkUtils::run_all_scenarios_test(SolverInterface* solver) {
    // 设置CPU亲和性到最空闲的核心
    setCpuAffinity();
    
    auto map_paths = get_all_map_paths();
    std::string root_dir = get_project_root();

    std::vector<std::string> listofmaps = {
        "Berlin_1_256",
        "Boston_0_256",
        "Paris_1_256",
        "den312d",
        "empty-48-48",
        "lak303d",
        "maze-32-32-2",
        "maze-128-128-1",
        "maze-128-128-10",
        "orz900d",
        "random-32-32-20",
        "random-64-64-10",
        "random-64-64-20",
        "room-32-32-4",
        "room-64-64-8",
        "room-64-64-16",
        "w_woundedcoast",
        "warehouse-10-20-10-2-1",
        "warehouse-10-20-10-2-2"
    };
    
    for (const auto& map_path : map_paths) {
        std::string map_name = get_map_name(map_path); // 提取地图名
        if (std::find(listofmaps.begin(), listofmaps.end(), map_name) == listofmaps.end()) {
            continue;
        }
        std::string solver_name = solver->get_name();
        std::string csv_filename = "benchmark_" + solver_name + "_" + map_name + ".csv";
        
        // 检查是否已经测试过
        fs::path results_dir = fs::path(root_dir) / "benchmark_results" / solver_name;
        fs::path csv_path = results_dir / csv_filename;
        if (fs::exists(csv_path)) {
            logger::log_info("Skip completed maps: " + map_name);
            continue;
        }
        
        // 确保solver目录存在
        if (!fs::exists(results_dir)) {
            logger::log_info("Creating solver directory: " + results_dir.string());
            if (!fs::create_directories(results_dir)) {
                throw std::runtime_error("Failed to create solver directory: " + results_dir.string());
            }
        }
        
        logger::log_info("Testing map: " + map_name);
        
        // 为当前地图重新进行预处理
        auto grid = load_map(map_path);
        logger::log_info("Preprocessing map: " + map_name);
        solver->preprocess(grid);
        logger::log_info("Preprocessing completed for map: " + map_name);
        
        // 测试even和random场景
        fs::path data_dir = fs::path(root_dir) / "data";
        fs::path even_scen_dir = data_dir / "mapf-scen-even";
        fs::path random_scen_dir = data_dir / "mapf-scen-random";
        
        std::vector<std::pair<std::string, fs::path>> scenario_types = {
            {"even", even_scen_dir},
            {"random", random_scen_dir}
        };
        
        std::vector<SingleAgentResult> map_results;
        for (const auto& [type_name, scen_dir] : scenario_types) {
            logger::log_info("Testing " + type_name + " scenarios");
            
            for (int i = 1; i <= 25; ++i) {
                std::string scen_file = make_scen_path(scen_dir.string(), map_name, type_name, i);
                
                if (fs::exists(scen_file)) {
                    // logger::log_info("Testing scenario: " + scen_file);
                    auto scenario_results = run_scen_file_test(map_path, scen_file, solver);
                    
                    // 保存到当前地图的结果中
                    map_results.insert(map_results.end(), 
                                    scenario_results.begin(), 
                                    scenario_results.end());
                } else {
                    logger::log_warning("Scenario file not found: " + scen_file);
                }
            }
        }
        
        // 写入当前地图的结果
        write_results_to_csv(csv_filename, map_results, solver_name);
        // 追加summary到同一个csv文件
        write_summary_to_csv(csv_filename, map_results, solver);
    }
    
    // 恢复线程优先级
    restore_thread_priority();
}

void BenchmarkUtils::run_single_scenario_test(
    const std::string& map_name,
    const std::string& scenario_type,
    int scenario_number,
    SolverInterface* solver) {
    
    try {
        // 设置CPU亲和性到最空闲的核心
        setCpuAffinity();
        
        // 构建文件路径
        std::string root_dir = get_project_root();
        fs::path data_dir = fs::path(root_dir) / "data";
        fs::path map_path = data_dir / "mapf-map" / (map_name + ".map");
        fs::path scen_dir = data_dir / ("mapf-scen-" + scenario_type);
        
        std::string scen_file = make_scen_path(
            scen_dir.string(), 
            map_name, 
            scenario_type, 
            scenario_number
        );
        
        // 检查文件是否存在
        if (!fs::exists(map_path)) {
            logger::log_error("Map file not found: " + map_path.string());
            return;
        }
        if (!fs::exists(scen_file)) {
            logger::log_error("Scen file not found: " + scen_file);
            return;
        }
        
        logger::log_info("Testing map: " + map_name);
        logger::log_info("Testing scenario: " + scen_file);
        
        // 为当前地图进行预处理
        auto grid = load_map(map_path.string());
        logger::log_info("Preprocessing map: " + map_name);
        solver->preprocess(grid);
        logger::log_info("Preprocessing completed for map: " + map_name);
        
        // 运行测试
        auto results = run_scen_file_test(map_path.string(), scen_file, solver, 30);
        
        // 输出结果
        for (const auto& result : results) {
            logger::log_info("\nTesting result:");
            logger::log_info("Start: (" + std::to_string(result.start.x) + "," + std::to_string(result.start.y) + ")");
            logger::log_info("Target: (" + std::to_string(result.target.x) + "," + std::to_string(result.target.y) + ")");
            logger::log_info("Success: " + std::string(result.success ? "Yes" : "No"));
            logger::log_info("Runtime: " + std::to_string(result.runtime) + " seconds");
            logger::log_info("Path length: " + std::to_string(result.path_length));
        }
        
        // 写入CSV结果文件
        std::string solver_name = solver->get_name();
        std::string csv_filename = solver_name + "_" + map_name + "_" + scenario_type + "_" + std::to_string(scenario_number) + ".csv";
        write_results_to_csv(csv_filename, results, solver_name);
        
        // 在同一个CSV文件末尾添加汇总统计
        write_summary_to_csv(csv_filename, results, solver);
        
        // 计算并输出统计信息
        auto stats = calculate_stats(results);
        stats.print();
        
        // 恢复线程优先级
        restore_thread_priority();
        
    } catch (const std::exception& e) {
        logger::log_error("Exception in single scenario test: " + std::string(e.what()));
        // 确保在异常情况下也恢复线程优先级
        restore_thread_priority();
    }
}

void BenchmarkUtils::run_single_agent_test(
    const std::string& map_name,
    const std::string& scenario_type,
    int scenario_number,
    int agent_index,
    SolverInterface* solver,
    double time_limit) {
    // 构建文件路径
    std::string root_dir = get_project_root();
    fs::path data_dir = fs::path(root_dir) / "data";
    fs::path map_path = data_dir / "mapf-map" / (map_name + ".map");
    fs::path scen_dir = data_dir / ("mapf-scen-" + scenario_type);
    
    std::string scen_file = make_scen_path(
        scen_dir.string(), 
        map_name, 
        scenario_type, 
        scenario_number
    );
    
    // 检查文件是否存在
    if (!fs::exists(map_path)) {
        logger::log_error("Map file not found: " + map_path.string());
        return;
    }
    if (!fs::exists(scen_file)) {
        logger::log_error("Scen file not found: " + scen_file);
        return;
    }
    
    logger::log_info("Testing map: " + map_name);
    logger::log_info("Testing scenario: " + scen_file);
    // 为当前地图进行预处理
    auto grid = load_map(map_path.string());
    // 加载场景
    auto agents = load_scen(scen_file, grid);


    logger::log_info("Preprocessing map: " + map_name);
    solver->preprocess(grid);
    logger::log_info("Preprocessing completed for map: " + map_name);

    
    if (agent_index < 0 || agent_index >= agents.size()) {
        logger::log_error("agent_index out of range");
        return;
    }

    const auto& agent = agents[agent_index];
    logger::log_info("testing agent " + std::to_string(agent_index) + "  from (" + std::to_string(agent.start.x) + "," + std::to_string(agent.start.y) + ")" +
                        " to (" + std::to_string(agent.goal.x) + "," + std::to_string(agent.goal.y) + ")");

    // 重置计数
    solver->resetExpandedNodes();
    solver->resetSearchTime();
    solver->resetSearchMemoryUsage();

    // 搜索
    auto path = solver->search(agent.start, agent.goal);

    // 输出结果
    if (!path.empty()) {
        logger::log_info("found path with length: " + std::to_string(path.size()-1));
    } else {
        logger::log_info("no path found");
    }

    // 输出内存信息
    double preprocess_mem_mb = static_cast<double>(solver->getPreprocessMemoryUsage()) / (1024 * 1024);
    double search_mem_inc_mb = static_cast<double>(solver->getSearchMemoryIncrease()) / (1024 * 1024);
    logger::log_info("preprocess memory(MB): " + std::to_string(preprocess_mem_mb));
    logger::log_info("search memory increase(MB): " + std::to_string(search_mem_inc_mb));
}

SingleAgentStats BenchmarkUtils::calculate_stats(const std::vector<SingleAgentResult>& results) {
    SingleAgentStats stats;
    stats.total_instances = results.size();
    stats.successful_instances = 0;
    
    double total_runtime = 0.0;
    double total_path_length = 0.0;
    int total_nodes = 0;
    int valid_nodes_count = 0;
    
    for (const auto& result : results) {
        if (result.success) {
            stats.successful_instances++;
            total_runtime += result.runtime;
            total_path_length += result.path_length;
            
            if (result.nodes_expanded >= 0) {
                total_nodes += result.nodes_expanded;
                valid_nodes_count++;
            }
        }
    }
    
    stats.success_rate = static_cast<double>(stats.successful_instances) / stats.total_instances;
    
    if (stats.successful_instances > 0) {
        stats.avg_runtime = total_runtime / stats.successful_instances;
        stats.avg_path_length = total_path_length / stats.successful_instances;
    } else {
        stats.avg_runtime = 0.0;
        stats.avg_path_length = 0.0;
    }
    
    if (valid_nodes_count > 0) {
        stats.avg_nodes_expanded = static_cast<double>(total_nodes) / valid_nodes_count;
    } else {
        stats.avg_nodes_expanded = 0.0;
    }
    
    return stats;
}

void BenchmarkUtils::write_results_to_csv(
    const std::string& filename,
    const std::vector<SingleAgentResult>& results,
    const std::string& solver_name) {
    
    try {
        auto file = create_csv_file(filename, solver_name);
        
        // 写入表头
        file << "Map,Scenario,StartX,StartY,TargetX,TargetY,Success,Runtime,PathLength,NodesExpanded,PreprocessMemory(MB),SearchMemoryIncrease(MB)\n";
        
        // 写入结果
        for (const auto& result : results) {
            double preprocess_memory_mb = static_cast<double>(result.memory_before) / (1024 * 1024);
            double search_memory_increase_mb = static_cast<double>(result.memory_after) / (1024 * 1024);
            
            file << result.map_name << ","
                 << result.scen_name << ","
                 << result.start.x << ","
                 << result.start.y << ","
                 << result.target.x << ","
                 << result.target.y << ","
                 << (result.success ? "Yes" : "No") << ","
                 << std::fixed << std::setprecision(6) << result.runtime << ","
                 << std::fixed << std::setprecision(2) << result.path_length << ","
                 << result.nodes_expanded << ","
                 << std::fixed << std::setprecision(2) << preprocess_memory_mb << ","
                 << std::fixed << std::setprecision(2) << search_memory_increase_mb << "\n";
        }
        
        file.close();
        logger::log_info("Results written to: " + filename);
        
    } catch (const std::exception& e) {
        logger::log_error("Error writing results to CSV: " + std::string(e.what()));
        throw;
    }
}

void BenchmarkUtils::write_summary_to_csv(
    const std::string& filename,
    const std::vector<SingleAgentResult>& all_results,
    SolverInterface* solver) {
    
    try {
        // 获取项目根目录
        std::string root_dir = get_project_root();
        fs::path results_dir = fs::path(root_dir) / "benchmark_results" / solver->get_name();
        
        // 构建完整的文件路径
        fs::path file_path = results_dir / filename;
        
        // 以追加模式打开文件
        std::ofstream file(file_path, std::ios::app);
        if (!file.is_open()) {
            throw std::runtime_error("Unable to append to CSV file: " + file_path.string());
        }
        
        auto stats = calculate_stats(all_results);
        
        // 直接获取solver的预处理时间（适用于单一solver/单一预处理场景）
        file << "\n" << solver->get_name() << " Summary Statistics:\n";
        file << "Total Instances,Successful Instances,Success Rate,Average Runtime,Average Path Length,Average Nodes Expanded,Preprocess Time(s)\n";
        file << stats.total_instances << ","
             << stats.successful_instances << ","
             << std::fixed << std::setprecision(2) << (stats.success_rate * 100) << "%,"
             << std::fixed << std::setprecision(6) << stats.avg_runtime << ","
             << std::fixed << std::setprecision(2) << stats.avg_path_length << ","
             << std::fixed << std::setprecision(2) << stats.avg_nodes_expanded << ","
             << std::fixed << std::setprecision(6) << solver->getPreprocessTime() << "\n";
        
        file.close();
        logger::log_info("Summary statistics appended to: " + filename);
        
    } catch (const std::exception& e) {
        logger::log_error("Error writing summary to CSV: " + std::string(e.what()));
        throw;
    }
}

void BenchmarkUtils::print_result(const SingleAgentResult& result) {
    std::cout << std::fixed << std::setprecision(4)
              << "(" << result.start.x << "," << result.start.y << ") -> ("
              << result.target.x << "," << result.target.y << "): "
              << (result.success ? "SUCCESS" : "FAILED") << " | "
              << "Time: " << result.runtime << "s | "
              << "Path Length: " << result.path_length
              << std::endl;
}

std::string BenchmarkUtils::get_project_root() {
    fs::path current_path = fs::current_path();
    while (!fs::exists(current_path / "data")) {
        if (current_path.parent_path() == current_path) {
            throw std::runtime_error("Project root directory not found");
        }
        current_path = current_path.parent_path();
    }
    return current_path.string();
}

std::vector<std::string> BenchmarkUtils::get_all_map_paths() {
    std::vector<std::string> map_paths;
    std::string root_dir = get_project_root();
    fs::path data_dir = fs::path(root_dir) / "data";
    fs::path maps_dir = data_dir / "mapf-map";
    
    if (!fs::exists(maps_dir)) {
        throw std::runtime_error("Map directory not found: " + maps_dir.string());
    }
    
    for (const auto& map_entry : fs::directory_iterator(maps_dir)) {
        if (map_entry.path().extension() == ".map") {
            map_paths.push_back(map_entry.path().string());
        }
    }
    
    return map_paths;
}

std::string BenchmarkUtils::get_map_name(const std::string& map_path) {
    fs::path path(map_path);
    return path.stem().string();
}

std::string BenchmarkUtils::make_scen_path(const std::string& scen_dir, 
                                                     const std::string& map_name,
                                                     const std::string& type,
                                                     int index) {
    return (fs::path(scen_dir) / 
            (map_name + "-" + type + "-" + std::to_string(index) + ".scen")).string();
}

double BenchmarkUtils::calculate_path_length(const std::vector<Vertex>& path) {
    if (path.size() < 2) return 0.0;
    
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        length += std::abs(path[i].x - path[i-1].x) + std::abs(path[i].y - path[i-1].y);
    }
    return length;
}

std::ofstream BenchmarkUtils::create_csv_file(const std::string& filename, const std::string& solver_name) {
    // 获取项目根目录
    std::string root_dir = get_project_root();
    fs::path results_dir = fs::path(root_dir) / "benchmark_results" / solver_name;
    
    // 确保目录存在
    if (!fs::exists(results_dir)) {
        logger::log_info("Creating directory: " + results_dir.string());
        if (!fs::create_directories(results_dir)) {
            throw std::runtime_error("Failed to create directory: " + results_dir.string());
        }
    }
    
    // 构建完整的文件路径
    fs::path file_path = results_dir / filename;
    logger::log_info("Writing results to: " + file_path.string());
    
    std::ofstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to create CSV file: " + file_path.string());
    }
    
    return file;
}



// 静态成员变量定义
DWORD_PTR BenchmarkUtils::original_affinity = 0;
int BenchmarkUtils::original_priority = 0;

void BenchmarkUtils::setCpuAffinity() {
    int core_count = getCpuCoreCount();
    if (core_count <= 1) {
        logger::log_info("Single core system, no affinity setting needed");
        return;
    }
    
    // 查找最空闲的CPU核心
    int target_core = find_least_busy_cpu();
    
    // 优化线程优先级
    optimize_thread_priority();
    
    // Windows实现
    DWORD_PTR mask = (1ULL << target_core);
    if (SetThreadAffinityMask(GetCurrentThread(), mask)) {
        logger::log_info("Set CPU affinity to core " + std::to_string(target_core) + " on Windows");
    } else {
        logger::log_warning("Failed to set CPU affinity on Windows");
    }
}

int BenchmarkUtils::getCpuCoreCount() {
    // Windows实现
    SYSTEM_INFO sysInfo;
    GetSystemInfo(&sysInfo);
    return sysInfo.dwNumberOfProcessors;
}

int BenchmarkUtils::find_least_busy_cpu() {
    SYSTEM_INFO sysinfo;
    GetSystemInfo(&sysinfo);
    int num_cpus = sysinfo.dwNumberOfProcessors;
    
    double min_load = (std::numeric_limits<double>::max)();
    int selected_cpu = 1;  // 默认从CPU 1开始，避免使用CPU 0
    
    // 收集每个CPU的负载
    for (int i = 1; i < num_cpus; ++i) {
        double load = get_cpu_load(i);
        if (load < min_load) {
            min_load = load;
            selected_cpu = i;
        }
    }
    
    logger::log_info("Selected CPU " + std::to_string(selected_cpu) + 
                    " with load " + std::to_string(min_load * 100) + "%");
    return selected_cpu;
}

double BenchmarkUtils::get_cpu_load(int cpu_id) {
    SYSTEM_INFO sysInfo;
    GetSystemInfo(&sysInfo);
    
    // 检查CPU ID是否有效
    if (cpu_id < 0 || cpu_id >= static_cast<int>(sysInfo.dwNumberOfProcessors)) {
        logger::log_error("Invalid CPU ID: " + std::to_string(cpu_id));
        return 0.0;
    }

    DWORD_PTR oldMask = SetThreadAffinityMask(GetCurrentThread(), (static_cast<DWORD_PTR>(1) << cpu_id));
    if (oldMask == 0) {
        logger::log_error("Failed to set thread affinity for CPU " + std::to_string(cpu_id));
        return 0.0;
    }

    FILETIME idleTime1, kernelTime1, userTime1;
    FILETIME idleTime2, kernelTime2, userTime2;

    // 获取第1次CPU时间
    if (!GetSystemTimes(&idleTime1, &kernelTime1, &userTime1)) {
        SetThreadAffinityMask(GetCurrentThread(), oldMask);
        logger::log_error("Failed to get first CPU times for CPU " + std::to_string(cpu_id));
        return 0.0;
    }

    // 等待100ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 获取第2次CPU时间
    if (!GetSystemTimes(&idleTime2, &kernelTime2, &userTime2)) {
        SetThreadAffinityMask(GetCurrentThread(), oldMask);
        logger::log_error("Failed to get second CPU times for CPU " + std::to_string(cpu_id));
        return 0.0;
    }

    // 恢复原来的线程亲和性
    SetThreadAffinityMask(GetCurrentThread(), oldMask);

    // 计算时间差
    ULONGLONG idle1 = (static_cast<ULONGLONG>(idleTime1.dwHighDateTime) << 32) | idleTime1.dwLowDateTime;
    ULONGLONG kernel1 = (static_cast<ULONGLONG>(kernelTime1.dwHighDateTime) << 32) | kernelTime1.dwLowDateTime;
    ULONGLONG user1 = (static_cast<ULONGLONG>(userTime1.dwHighDateTime) << 32) | userTime1.dwLowDateTime;

    ULONGLONG idle2 = (static_cast<ULONGLONG>(idleTime2.dwHighDateTime) << 32) | idleTime2.dwLowDateTime;
    ULONGLONG kernel2 = (static_cast<ULONGLONG>(kernelTime2.dwHighDateTime) << 32) | kernelTime2.dwLowDateTime;
    ULONGLONG user2 = (static_cast<ULONGLONG>(userTime2.dwHighDateTime) << 32) | userTime2.dwLowDateTime;

    ULONGLONG idleDiff = idle2 - idle1;
    ULONGLONG kernelDiff = kernel2 - kernel1;
    ULONGLONG userDiff = user2 - user1;

    ULONGLONG totalDiff = kernelDiff + userDiff;
    if (totalDiff == 0) {
        logger::log_warning("Zero total time difference for CPU " + std::to_string(cpu_id));
        return 0.0;
    }

    // 计算CPU使用率（返回0-1之间的值）
    double cpuUsage = 1.0 - (static_cast<double>(idleDiff) / totalDiff);
    
    // logger::log_info("CPU " + std::to_string(cpu_id) + " load: " + 
    //                 std::to_string(cpuUsage * 100) + "%");
    
    return cpuUsage;
}

void BenchmarkUtils::optimize_thread_priority() {
    // 保存原始设置
    HANDLE thread = GetCurrentThread();
    original_affinity = SetThreadAffinityMask(thread, 0);
    original_priority = GetThreadPriority(thread);
    
    // 设置为高优先级
    SetThreadPriority(thread, THREAD_PRIORITY_HIGHEST);
    
    // 设置进程优先级
    HANDLE process = GetCurrentProcess();
    SetPriorityClass(process, HIGH_PRIORITY_CLASS);
}

void BenchmarkUtils::restore_thread_priority() {
    HANDLE thread = GetCurrentThread();
    SetThreadAffinityMask(thread, original_affinity);
    SetThreadPriority(thread, original_priority);
    
    HANDLE process = GetCurrentProcess();
    SetPriorityClass(process, NORMAL_PRIORITY_CLASS);
} 