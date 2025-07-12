#include "JPS.h"
#include "../utilities/GridUtility.h"
#include "../utilities/Log.h"
#include "../heuristic/Heuristic.h"
#include "../astar/AStar.h"
#include "../action/Action.h"
#include <algorithm>
#include <unordered_map>
#include <sstream>
#include <chrono>
#include <queue>

namespace {
    inline bool is_cycle_free(const std::shared_ptr<AStarNode>& current, const Vertex& point) {
        // 检查从parent到current的完整路径
        if (!current->parent) return true;
        
        // 获取从parent到current的完整路径
        auto parent = current->parent;
        Vertex dir = utils::calculateDirection(parent->pos, current->pos);
        Vertex pos = parent->pos;
        
        // 检查路径上的每个点
        while (pos != current->pos) {
            if (pos == point) {
                return false;  // 发现环路
            }
            pos = Vertex(pos.x + dir.x, pos.y + dir.y);
        }
        
        // 递归检查parent的路径
        return is_cycle_free(parent, point);
    }

    std::vector<Vertex> check_diagonal_forced(const std::vector<std::vector<int>>& grid, int x, int y, int dx, int dy) {
        std::vector<Vertex> forced_neighbors;
        Vertex from(x,y);
        Vertex neighbour1(x - dx, y);
        Vertex forced_neighbour1(x - dx, y + dy);
        Vertex neighbour2(x, y - dy);
        Vertex forced_neighbour2(x + dx, y - dy);
        
        if (!utils::isWalkable(grid, from, neighbour1) && utils::isWalkable(grid, from, forced_neighbour1)) {
            forced_neighbors.push_back(Vertex(-dx, dy));
        }

        if (!utils::isWalkable(grid, from, neighbour2) && utils::isWalkable(grid, from, forced_neighbour2)) {
            forced_neighbors.push_back(Vertex(dx, -dy));
        }

        return forced_neighbors;
    }

    std::vector<Vertex> check_straight_forced(const std::vector<std::vector<int>>& grid, int x, int y, int dx, int dy) {
        std::vector<Vertex> forced_neighbors;
        Vertex from(x, y);
        
        if (dx != 0) {  // 垂直移动（南/北）
            Vertex neighbour1(x, y - 1);  // 西边邻居
            Vertex forced_neighbour1(x + dx, y - 1);  // 西边的强制邻居
            
            if (!utils::isWalkable(grid, from, neighbour1) && 
                utils::isWalkable(grid, from, forced_neighbour1)) {
                forced_neighbors.push_back(Vertex(dx, -1));
            }
            
            Vertex neighbour2(x, y + 1);  // 东边邻居
            Vertex forced_neighbour2(x + dx, y + 1);  // 东边的强制邻居
            
            if (!utils::isWalkable(grid, from, neighbour2) && 
                utils::isWalkable(grid, from, forced_neighbour2)) {
                forced_neighbors.push_back(Vertex(dx, 1));
            }
            
        } else {  // 水平移动（东/西）
            Vertex neighbour1(x - 1, y);  // 北边邻居
            Vertex forced_neighbour1(x - 1, y + dy);  // 北边的强制邻居
            
            if (!utils::isWalkable(grid, from, neighbour1) && 
                utils::isWalkable(grid, from, forced_neighbour1)) {
                forced_neighbors.push_back(Vertex(-1, dy));
            }
            
            Vertex neighbour2(x + 1, y);  // 南边邻居
            Vertex forced_neighbour2(x + 1, y + dy);  // 南边的强制邻居
            
            if (!utils::isWalkable(grid, from, neighbour2) && 
                utils::isWalkable(grid, from, forced_neighbour2)) {
                forced_neighbors.push_back(Vertex(1, dy));
            }
        }
        
        return forced_neighbors;
    }
    
    std::vector<Vertex> get_pruned_neighbors(const std::shared_ptr<AStarNode>& current,
                             const std::vector<std::vector<int>>& grid) {
        std::vector<Vertex> neighbors;
        const Vertex& pos = current->pos;
        
        // If starting node, explore all 8 directions
        if (!current->parent) {
            for (const auto& move : Action::DIRECTIONS_8) {
                if (utils::isWalkable(grid, pos, Vertex(pos.x + move.x, pos.y + move.y))) {
                    neighbors.push_back(move);
                }
            }
            return neighbors;
        }
    
        // Calculate incoming direction
        Vertex dir = utils::calculateDirection(current->parent->pos, current->pos);

        // 1. Current diagonal direction
        if (utils::isWalkable(grid, pos, Vertex(pos.x + dir.x, pos.y + dir.y))) {
                neighbors.push_back(dir);
            }
        
        // Forced neighbors
        if (utils::isDiagonal(dir)) {
            if (utils::isWalkable(grid, pos, Vertex(pos.x + dir.x, pos.y))) {
                neighbors.push_back(Vertex(dir.x, 0));
            }
            if (utils::isWalkable(grid, pos, Vertex(pos.x, pos.y + dir.y))) {
                neighbors.push_back(Vertex(0, dir.y));
            }
            auto forced = check_diagonal_forced(grid, pos.x, pos.y, dir.x, dir.y);
            neighbors.insert(neighbors.end(), forced.begin(), forced.end());

        } else {
            auto forced = check_straight_forced(grid, pos.x, pos.y, dir.x, dir.y);
            neighbors.insert(neighbors.end(), forced.begin(), forced.end());
        }
        
        return neighbors;
    }

    Vertex jump(int x, int y, int dx, int dy, const std::shared_ptr<AStarNode>& current,
                const std::vector<std::vector<int>>& grid, const Vertex& goal) {
                
        // 1: n ← step(x, d~)
        Vertex next(x + dx, y + dy);
    
        // 构建基础日志信息
        std::stringstream base_info;
        base_info << "From (" << x << "," << y << ") with direction (" << dx << "," << dy << ")";

        // 2: if n is an obstacle or is outside the grid then
        // 3: return null
        if (!utils::isWalkable(grid, Vertex(x,y), next)) {
            return Vertex(-1, -1);
        }

        // 4: if n = g then
        // 5: return n
        if (next == goal) {
            return next;
        }

        if (!is_cycle_free(current, next)) {
            return Vertex(-1, -1);
        }
    
        // 6: if ∃ n′ ∈ neighbours(n) s.t. n′ is forced then
        // 7: return n
        if (dx != 0 && dy != 0) {  // Diagonal move
            if(!check_diagonal_forced(grid, next.x, next.y, dx, dy).empty()) {
                return next;
            }
            // 8: if d~ is diagonal then
            // 9: for all i ∈ {1, 2} do
            // 10: if jump(n, d~i, s, g) is not null then
            // 11: return n
            if (jump(next.x, next.y, dx, 0, current, grid, goal).x != -1 ||
                jump(next.x, next.y, 0, dy, current, grid, goal).x != -1) {
                return next;
            }
        } else {
            if(!check_straight_forced(grid, next.x, next.y, dx, dy).empty()) {
                return next;
            }
        }
    
        // 12: return jump(n, d~, s, g)
        return jump(next.x, next.y, dx, dy, current, grid, goal);
    }

    std::vector<Vertex> identify_successors(const std::shared_ptr<AStarNode>& current,
                                      const std::vector<std::vector<int>>& grid,
                                      const Vertex& goal) {
        // 1: successors(x) ← ∅
        std::vector<Vertex> successors;

        // 2: neighbours(x) ← prune(x, neighbours(x))
        std::vector<Vertex> pruned_dirs = get_pruned_neighbors(current, grid);
        // logger::log_info("Pruned directions: " + logger::vectorToString(pruned_dirs));

        // 3: for all n ∈ neighbours(x) do
        for (const auto& dir : pruned_dirs) {
            // 4: n ← jump(x, direction(x, n), s, g)
            Vertex jump_point = jump(current->pos.x, current->pos.y, 
                                   dir.x, dir.y, current, grid, goal);

            // 5: add n to successors(x)
            if (jump_point.x != -1) {
                successors.push_back(jump_point);
                // logger::log_info("Jump point found: (" + std::to_string(jump_point.x) + "," + 
                //                std::to_string(jump_point.y) + ") - added to successors");
            }
        }

        // 6: return successors(x)
        return successors;
    }

    
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


std::vector<Vertex> jump_point_search(const Vertex& start, const Vertex& goal,
                         const std::vector<std::vector<int>>& grid) {
    // 记录开始时间
    auto start_time = std::chrono::steady_clock::now();
    const int MAX_SEARCH_TIME = 30;  // 最大搜索时间（秒）

    // 检查起点和终点是否可通行
    if(!utils::isPassable(grid, start) || !utils::isPassable(grid, goal)) {
        return {};
    }

    std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>, AStarNodeComparator> open_list;
    std::unordered_map<Vertex, double> closed_list;  // 改用unordered_map存储已访问节点

    // 创建并添加起始节点
    auto start_node = std::make_shared<AStarNode>(start, 0, heuristic(start, goal));
    open_list.push(start_node);

    while (!open_list.empty()) {
        // 检查是否超时
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        if (elapsed >= MAX_SEARCH_TIME) {
            logger::log_warning("JPS search timed out after " + std::to_string(MAX_SEARCH_TIME) + " seconds");
            return {};
        }

        auto current = open_list.top();
        open_list.pop();

        // logger::log_info("Current node: (" + std::to_string(current->pos.x) + "," + 
        //                std::to_string(current->pos.y) + "), g-value: " + 
        //                std::to_string(current->g) + ", h-value: " + 
        //                std::to_string(current->h));

        if (current->pos == goal) {
            return reconstruct_path(current);
        }
        
        // identify_successors
        std::vector<Vertex> successors = identify_successors(current, grid, goal);
        
        // Expand current node
        for (const auto& successor : successors) {
            double move_cost = utils::getMoveCost(current->pos, successor);
            double tentative_g = current->g + move_cost;

            // 检查节点是否在closed list中且新路径不会更优
            if (closed_list.count(successor) && closed_list[successor] <= tentative_g) {
                continue;
            }

            // 更新或添加到closed list
            closed_list[successor] = tentative_g;

            double h_value = heuristic(successor, goal);
            auto next_node = std::make_shared<AStarNode>(
                successor, 
                tentative_g,
                h_value,
                current
            );
            // logger::log_info("Generated successor node: (" + std::to_string(successor.x) + "," + 
            //                std::to_string(successor.y) + "), g-value: " + 
            //                std::to_string(tentative_g) + ", movement cost: " + 
            //                std::to_string(move_cost) + ", h-value: " + 
            //                std::to_string(h_value));
            // auto node = current;
            
            // std::stringstream ss;   
            // ss << "parents: ";
            // while (node) {
            //     ss << "(" << node->pos.x << "," << node->pos.y << ")";
            //     node = node->parent;
            // }
            // logger::log_info(ss.str());
            
            open_list.push(next_node);
        }
    }

    logger::log_warning("JPS found no path from (" + 
                      std::to_string(start.x) + "," + std::to_string(start.y) + 
                      ") to (" + std::to_string(goal.x) + "," + std::to_string(goal.y) + ")");
    return {};
}





