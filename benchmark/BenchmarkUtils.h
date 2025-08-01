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

// single agent test result structure
struct SingleAgentResult {
    bool success;                    // whether path is found
    double runtime;                  // running time (seconds)
    double path_length;              // path length
    int nodes_expanded;              // number of expanded nodes
    size_t memory_before;            // preprocessing memory usage (bytes)
    size_t memory_after;             // memory increase during search (bytes)
    std::string map_name;            // map name
    std::string scen_name;           // scenario name
    Vertex start;                    // start point
    Vertex target;                   // target point
};

// statistics information structure
struct SingleAgentStats {
    int total_instances;             // total number of instances
    int successful_instances;        // number of successful instances
    double avg_runtime;              // average running time
    double avg_path_length;          // average path length
    double avg_nodes_expanded;       // average number of expanded nodes
    double success_rate;             // success rate
    
    void print() const;
};

class BenchmarkUtils {
private:
    // save original settings for restoration
    static DWORD_PTR original_affinity;
    static int original_priority;
    
public:
    // run test for single scenario file
    static std::vector<SingleAgentResult> run_scen_file_test(
        const std::string& map_file,
        const std::string& scen_file,
        SolverInterface* solver,
        double time_limit = 30.0);
    
    // set CPU affinity to the least busy core
    static void setCpuAffinity();
    
    // get number of CPU cores
    static int getCpuCoreCount();
    
    // find least busy CPU core
    static int find_least_busy_cpu();
    
    // get load of specified CPU
    static double get_cpu_load(int cpu_id);
    
    // optimize thread priority
    static void optimize_thread_priority();
    
    // restore thread priority
    static void restore_thread_priority();
    
    // run test for all scenarios
    static void run_all_scenarios_test(SolverInterface* solver);
    
    // run test for single scenario (for debugging)
    static void run_single_scenario_test(
        const std::string& map_name,
        const std::string& scenario_type,
        int scenario_number,
        SolverInterface* solver);
    
    // run test for single start/goal pair
    static void run_single_agent_test(
        const std::string& map_name,
        const std::string& scenario_type,
        int scenario_number,
        int agent_index,
        SolverInterface* solver,
        double time_limit = 30.0);
    
    // calculate statistics
    static SingleAgentStats calculate_stats(const std::vector<SingleAgentResult>& results);
    
    // write results to CSV file
    static void write_results_to_csv(
        const std::string& filename,
        const std::vector<SingleAgentResult>& results,
        const std::string& solver_name);
    
    // write summary statistics to CSV file
    static void write_summary_to_csv(
        const std::string& filename,
        const std::vector<SingleAgentResult>& all_results,
        SolverInterface* solver);
    
    // print single result
    static void print_result(const SingleAgentResult& result);
    
    // get project root directory
    static std::string get_project_root();
    
    // get all map paths
    static std::vector<std::string> get_all_map_paths();
    
    // get map name
    static std::string get_map_name(const std::string& map_path);
    
    // build scenario file path
    static std::string make_scen_path(const std::string& scen_dir, 
                                     const std::string& map_name,
                                     const std::string& type,
                                     int index);

private:
    // create CSV file and return file stream
    static std::ofstream create_csv_file(const std::string& filename, const std::string& solver_name);
};

#endif // BENCHMARK_UTILS_H 