#ifdef _WIN32
#include <windows.h>
#endif
#ifndef BENCHMARK_UTILS_H
#define BENCHMARK_UTILS_H

#include "../src/KIAstar/KeyIntervalAStar.h"
#include "../src/KIAstar/Preprocess.h"
#include "../src/astar/AStar.h"
#include "../src/hpastar/HPAStar.h"
#include "../src/solver/SolverInterface.h"
#include "../data_loader/MapLoader.h"
#include "../src/utilities/Log.h"
#include <chrono>
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <string>
#include <vector>
#include <fstream>
#include <functional>

namespace fs = std::filesystem;

// 单智能体测试结果结构
struct SingleAgentResult {
    bool success;                    // 是否找到路径
    double runtime;                  // 运行时间（秒）
    double path_length;              // 路径长度
    int nodes_expanded;              // 扩展节点数
    size_t memory_before;            // 预处理内存使用量（字节）
    size_t memory_after;             // 搜索过程内存增加量（字节）
    std::string map_name;            // 地图名称
    std::string scen_name;           // 场景名称
    Vertex start;                    // 起点
    Vertex target;                   // 终点
};

// 统计信息结构
struct SingleAgentStats {
    int total_instances;             // 总实例数
    int successful_instances;        // 成功实例数
    double avg_runtime;              // 平均运行时间
    double avg_path_length;          // 平均路径长度
    double avg_nodes_expanded;       // 平均扩展节点数
    double success_rate;             // 成功率
    
    void print() const;
};

class BenchmarkUtils {
private:
    // 保存原始设置用于恢复
    static DWORD_PTR original_affinity;
    static int original_priority;
    
public:
    // 运行单个场景文件的测试
    static std::vector<SingleAgentResult> run_scen_file_test(
        const std::string& map_file,
        const std::string& scen_file,
        SolverInterface* solver,
        double time_limit = 30.0);
    
    // 设置CPU亲和性到最空闲的核心
    static void setCpuAffinity();
    
    // 获取系统CPU核心数
    static int getCpuCoreCount();
    
    // 查找最空闲的CPU核心
    static int find_least_busy_cpu();
    
    // 获取指定CPU的负载
    static double get_cpu_load(int cpu_id);
    
    // 优化线程优先级
    static void optimize_thread_priority();
    
    // 恢复线程优先级
    static void restore_thread_priority();
    
    // 运行所有场景的测试
    static void run_all_scenarios_test(SolverInterface* solver);
    
    // 运行单个场景的测试（用于调试）
    static void run_single_scenario_test(
        const std::string& map_name,
        const std::string& scenario_type,
        int scenario_number,
        SolverInterface* solver);
    
    // 运行单对start/goal的测试
    static void run_single_agent_test(
        const std::string& map_name,
        const std::string& scenario_type,
        int scenario_number,
        int agent_index,
        SolverInterface* solver,
        double time_limit = 30.0);
    
    // 计算统计信息
    static SingleAgentStats calculate_stats(const std::vector<SingleAgentResult>& results);
    
    // 写入结果到CSV文件
    static void write_results_to_csv(
        const std::string& filename,
        const std::vector<SingleAgentResult>& results,
        const std::string& solver_name);
    
    // 写入汇总统计到CSV文件
    static void write_summary_to_csv(
        const std::string& filename,
        const std::vector<SingleAgentResult>& all_results,
        SolverInterface* solver);
    
    // 打印单个结果
    static void print_result(const SingleAgentResult& result);
    
    // 获取项目根目录
    static std::string get_project_root();
    
    // 获取所有地图路径
    static std::vector<std::string> get_all_map_paths();
    
    // 获取地图名称
    static std::string get_map_name(const std::string& map_path);
    
    // 构建场景文件路径
    static std::string make_scen_path(const std::string& scen_dir, 
                                     const std::string& map_name,
                                     const std::string& type,
                                     int index);

private:
    // 计算路径长度
    static double calculate_path_length(const std::vector<Vertex>& path);
    
    // 创建CSV文件并返回文件流
    static std::ofstream create_csv_file(const std::string& filename, const std::string& solver_name);
};

#endif // BENCHMARK_UTILS_H 