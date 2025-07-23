#include "AStar.h"
#include "../utilities/Log.h"
#include "../heuristic/Heuristic.h"
#include "../action/Action.h"
#include <chrono> 
#include <queue> 
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <functional>
#include <windows.h>

namespace {
    const int MAX_SEARCH_TIME = 30;  // 最大搜索时间（秒）
    
    std::vector<Vertex> reconstruct_path(const std::shared_ptr<AStarNode>& goal_node) {
        std::vector<Vertex> path;
        auto node = goal_node;
        while (node) {
            path.push_back(node->pos);
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
}

// AStar类的实现
std::vector<Vertex> AStar::search(const Vertex& start, const Vertex& target) {
    // 重置计数
    expanded_nodes_ = 0;
    search_time_ = 0.0;
    
    // 使用QueryPerformanceCounter计时
    LARGE_INTEGER freq, t_start, t_end;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&t_start);

    if(!utils::isPassable(grid_, start) || !utils::isPassable(grid_, target)) {
        QueryPerformanceCounter(&t_end);
        search_time_ = static_cast<double>(t_end.QuadPart - t_start.QuadPart) / freq.QuadPart;
        return {};
    }
    
    std::priority_queue<std::shared_ptr<AStarNode>, 
                       std::vector<std::shared_ptr<AStarNode>>, 
                       AStarNodeComparator> open_list;
    std::unordered_map<Vertex, double> closed_list;

    memory_before_ = memory_utils::get_current_memory_usage();

    auto start_node = std::make_shared<AStarNode>(start, 0, heuristic(start, target));
    open_list.push(start_node);

    while (!open_list.empty()) {
        // 检查是否超时
        // auto current_time = std::chrono::steady_clock::now();
        // auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        // if (elapsed >= MAX_SEARCH_TIME) {
        //     logger::log_warning("A* search timed out after " + std::to_string(MAX_SEARCH_TIME) + " seconds");
        //     auto end_time = std::chrono::steady_clock::now();
        //     search_time_ = std::chrono::duration<double>(end_time - start_time).count();
        //     return {};
        // }

        auto current = open_list.top();
        open_list.pop();
        
        // 增加扩展节点计数
        expanded_nodes_++;

        if (current->pos == target) {
            auto path = reconstruct_path(current);
            QueryPerformanceCounter(&t_end);
            search_time_ = static_cast<double>(t_end.QuadPart - t_start.QuadPart) / freq.QuadPart;
            memory_after_ = memory_utils::get_current_memory_usage();
            return path;
        }

        for (const auto& move : Action::MOVEMENTS_4) {
            Vertex next_pos(current->pos.x + move.x, current->pos.y + move.y);

            if (!utils::isWalkable(grid_, current->pos, next_pos)) {
                continue;
            }

            double tentative_g;
            // if (utils::isDiagonal(next_pos - current->pos)) {
            //     tentative_g = current->g + std::sqrt(2.0);
            // } else {
                tentative_g = current->g + 1.0;
            // }

            if (closed_list.count(next_pos) && closed_list[next_pos] <= tentative_g) {
                continue;
            }

            closed_list[next_pos] = tentative_g;

            auto next_node = std::make_shared<AStarNode>(
                next_pos, tentative_g, heuristic(next_pos, target), current);
            
            open_list.push(next_node);
        }
    }

    QueryPerformanceCounter(&t_end);
    search_time_ = static_cast<double>(t_end.QuadPart - t_start.QuadPart) / freq.QuadPart;
    logger::log_info("No path found");
    memory_after_ = memory_utils::get_current_memory_usage();
    return {};
}

std::vector<Vertex> AStar::reconstruct_path(const std::shared_ptr<AStarNode>& goal_node) const {
    std::vector<Vertex> path;
    auto node = goal_node;
    while (node) {
        path.push_back(node->pos);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal,
                          const std::vector<std::vector<int>>& grid) {
    // 记录开始时间
    auto start_time = std::chrono::steady_clock::now();

    if(!utils::isPassable(grid, start) || !utils::isPassable(grid, goal)) {
        return {};
    }
    std::priority_queue<std::shared_ptr<AStarNode>, 
                       std::vector<std::shared_ptr<AStarNode>>, 
                       AStarNodeComparator> open_list;
    std::unordered_map<Vertex, double> closed_list;


    auto start_node = std::make_shared<AStarNode>(start, 0, heuristic(start, goal));
    open_list.push(start_node);

    while (!open_list.empty()) {
        // 检查是否超时
        // auto current_time = std::chrono::steady_clock::now();
        // auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        // if (elapsed >= MAX_SEARCH_TIME) {
        //     logger::log_warning("A* search timed out after " + std::to_string(MAX_SEARCH_TIME) + " seconds");
        //     return {};
        // }

        auto current = open_list.top();
        open_list.pop();

        // logger::log_info("Current node: (" + std::to_string(current->pos.x) + "," +
        //                std::to_string(current->pos.y) + "), g-value: " +
        //                std::to_string(current->g) + ", h-value: " +
        //                std::to_string(current->h));  

        if (current->pos == goal) {
            auto path = reconstruct_path(current);
            // logger::log_info("Path found: " + logger::vectorToString(path));
            // double cost = utils::calculate_path_cost(path);
            // logger::log_info("Cost: " + std::to_string(cost));
            return path;
        }

        for (const auto& move : Action::MOVEMENTS_4) {
            Vertex next_pos(current->pos.x + move.x, current->pos.y + move.y);

            if (!utils::isWalkable(grid, current->pos, next_pos)) {
                continue;
            }

            double tentative_g;
            // if (utils::isDiagonal(next_pos - current->pos)) {
            //     tentative_g = current->g + std::sqrt(2.0);
            // } else {
                tentative_g = current->g + 1.0;
            // }

            if (closed_list.count(next_pos) && closed_list[next_pos] <= tentative_g) {
                continue;
            }

            closed_list[next_pos] = tentative_g;

            auto next_node = std::make_shared<AStarNode>(
                next_pos, tentative_g, heuristic(next_pos, goal), current);
            
            // logger::log_info("Generated successor node: (" + std::to_string(neighbor.x) + "," +
            //                std::to_string(neighbor.y) + "), g-value: " +
            //                std::to_string(tentative_g) +
            //                ", h-value: " + std::to_string(heuristic(neighbor, goal)));
            
            open_list.push(next_node);
        }
    }

    // logger::log_info("No path found");
    return {};
}

// 支持自定义方向的A*搜索
std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal,
                          const std::vector<std::vector<int>>& grid,
                          const std::vector<Vertex>& directions) {
    auto start_time = std::chrono::steady_clock::now();

    if(!utils::isPassable(grid, start) || !utils::isPassable(grid, goal)) {
        return {};
    }
    std::priority_queue<std::shared_ptr<AStarNode>, 
                       std::vector<std::shared_ptr<AStarNode>>, 
                       AStarNodeComparator> open_list;
    std::unordered_map<Vertex, double> closed_list;

    auto start_node = std::make_shared<AStarNode>(start, 0, heuristic(start, goal));
    open_list.push(start_node);

    while (!open_list.empty()) {
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        if (elapsed >= MAX_SEARCH_TIME) {
            logger::log_warning("A* search timed out after " + std::to_string(MAX_SEARCH_TIME) + " seconds");
            return {};
        }

        auto current = open_list.top();
        open_list.pop();

        if (current->pos == goal) {
            auto path = reconstruct_path(current);
            return path;
        }

        for (const auto& move : directions) {
            Vertex next_pos(current->pos.x + move.x, current->pos.y + move.y);

            if (!utils::isWalkable(grid, current->pos, next_pos)) {
                continue;
            }

            double tentative_g;
            if (utils::isDiagonal(next_pos - current->pos)) {
                tentative_g = current->g + std::sqrt(2.0);
            } else {
                tentative_g = current->g + 1.0;
            }

            if (closed_list.count(next_pos) && closed_list[next_pos] <= tentative_g) {
                continue;
            }

            closed_list[next_pos] = tentative_g;

            auto next_node = std::make_shared<AStarNode>(
                next_pos, tentative_g, heuristic(next_pos, goal), current);
            open_list.push(next_node);
        }
    }

    logger::log_info("No path found");
    return {};
}


size_t AStar::getSearchMemoryIncrease() const {
    if (memory_after_ >= memory_before_) {
        return memory_after_ - memory_before_;
    } else {
        return 0;
    }
}

void AStar::resetSearchMemoryUsage() {
    memory_before_ = 0;
    memory_after_ = 0;
}