#include "HPAStar.h"
#include "../astar/AStar.h"
#include "../action/Action.h"
#include "../heuristic/Heuristic.h"
#include "../utilities/GridUtility.h"
#include "../utilities/Log.h"
#include <chrono>
#include <algorithm>
#include <cmath>

void HPAStar::preprocess(const std::vector<std::vector<int>>& grid) {
    //logger::log_info("Preprocessing HPAStar");
    auto start_time = std::chrono::high_resolution_clock::now();
    
    clusters_.clear();
    abstract_edges_.clear();
    
    // 记录预处理前内存使用
    size_t memory_before = memory_utils::get_current_memory_usage();
    
    grid_ = grid;

    //logger::log_info("Creating clusters");
    // 创建聚类
    create_clusters();
    
    //logger::log_info("Identifying entrances and exits");
    // 识别入口和出口点
    identify_entrances_and_exits();
    
    //logger::log_info("Computing abstract edges");
    // 计算抽象边
    compute_abstract_edges();
    
    // 记录预处理后内存使用
    size_t memory_after = memory_utils::get_current_memory_usage();
    preprocess_memory_ = memory_after - memory_before;
    //logger::log_info("Preprocess memory usage: " + std::to_string(preprocess_memory_));
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    preprocess_time_ = duration.count() / 1000000.0;
    //logger::log_info("Preprocessing completed");
}

void HPAStar::create_clusters() {
    if (grid_.empty() || grid_[0].empty()) return;
    
    int rows = grid_.size();    // 行数（row）
    int cols = grid_[0].size(); // 列数（col）
    int cluster_id = 0;
    // 按行(row)和列(col)分块
    for (int row = 0; row < rows; row += cluster_size_) {
        for (int col = 0; col < cols; col += cluster_size_) {
            Vertex top_left(row, col); // x=row, y=col
            Vertex bottom_right(std::min(row + cluster_size_ - 1, rows - 1), 
                               std::min(col + cluster_size_ - 1, cols - 1));
            // 只创建包含可通行点的聚类
            bool has_traversable = false;
            for (int r = top_left.x; r <= bottom_right.x && !has_traversable; ++r) {
                for (int c = top_left.y; c <= bottom_right.y && !has_traversable; ++c) {
                    if (utils::isPassable(grid_, Vertex(r, c))) {
                        has_traversable = true;
                    }
                }
            }
            if (has_traversable) {
                clusters_.emplace_back(cluster_id++, top_left, bottom_right);
            }
        }
    }
}

void HPAStar::identify_entrances_and_exits() {
    // 先清空所有exits
    for (auto& cluster : clusters_) {
        cluster.exits.clear();
    }
    int rows = grid_.size();    // 行数（row）
    int cols = grid_[0].size(); // 列数（col）
    // 只处理右边界和下边界
    for (auto& cluster : clusters_) {
        // 右边界：固定列，行递增
        std::vector<Vertex> boundary_points;
        int right_col = cluster.bottom_right.y; // y 实际为 col
        for (int row = cluster.top_left.x; row <= cluster.bottom_right.x; ++row) { // x 实际为 row
            Vertex v(row, right_col); // Vertex(x=row, y=col)
            if (utils::isPassable(grid_, v) && utils::isPassable(grid_, Vertex(row, right_col + 1))) {
                boundary_points.push_back(v);
            } else if (!boundary_points.empty()) {
                // 处理一个连续区间
                auto exits = generate_exits_from_boundary(boundary_points);
                for (const auto& exit : exits) {
                    cluster.exits.push_back(exit);
                    Cluster* right_cluster = find_cluster_by_position(exit.x, exit.y + 1);
                    if (right_cluster) right_cluster->exits.push_back(Vertex(exit.x, exit.y + 1));
                }
                boundary_points.clear();
            }
        }
        // 处理最后一组
        if (!boundary_points.empty()) {
            auto exits = generate_exits_from_boundary(boundary_points);
            for (const auto& exit : exits) {
                cluster.exits.push_back(exit);
                Cluster* right_cluster = find_cluster_by_position(exit.x, exit.y + 1);
                if (right_cluster) right_cluster->exits.push_back(Vertex(exit.x, exit.y + 1));
            }
            boundary_points.clear();
        }
        // 下边界：固定行，列递增
        boundary_points.clear();
        int bottom_row = cluster.bottom_right.x; // x 实际为 row
        for (int col = cluster.top_left.y; col <= cluster.bottom_right.y; ++col) { // y 实际为 col
            Vertex v(bottom_row, col);
            if (utils::isPassable(grid_, v) && bottom_row < rows - 1 && utils::isPassable(grid_, Vertex(bottom_row + 1, col))) {
                boundary_points.push_back(v);
            } else if (!boundary_points.empty()) {
                auto exits = generate_exits_from_boundary(boundary_points);
                for (const auto& exit : exits) {
                    cluster.exits.push_back(exit);
                    Cluster* bottom_cluster = find_cluster_by_position(exit.x + 1, exit.y);
                    if (bottom_cluster) bottom_cluster->exits.push_back(Vertex(exit.x + 1, exit.y));
                }
                boundary_points.clear();
            }
        }
        if (!boundary_points.empty()) {
            auto exits = generate_exits_from_boundary(boundary_points);
            for (const auto& exit : exits) {
                cluster.exits.push_back(exit);
                Cluster* bottom_cluster = find_cluster_by_position(exit.x + 1, exit.y);
                if (bottom_cluster) bottom_cluster->exits.push_back(Vertex(exit.x + 1, exit.y));
            }
            boundary_points.clear();
        }
    }
    // 最后对每个cluster的exits去重
    for (auto& cluster : clusters_) {
        std::unordered_set<std::string> exit_set;
        std::vector<Vertex> unique_exits;
        for (const auto& v : cluster.exits) {
            std::string key = std::to_string(v.x) + "," + std::to_string(v.y);
            if (exit_set.find(key) == exit_set.end()) {
                exit_set.insert(key);
                unique_exits.push_back(v);
            }
        }
        cluster.exits = std::move(unique_exits);
    }
}

// 辅助：根据坐标查找cluster指针
Cluster* HPAStar::find_cluster_by_position(int x, int y) {
    for (auto& cluster : clusters_) {
        if (cluster.contains(Vertex(x, y))) {
            return &cluster;
        }
    }
    return nullptr;
}


std::vector<Vertex> HPAStar::generate_exits_from_boundary(const std::vector<Vertex>& boundary) const {
    const int WIDTH_THRESHOLD = 6;
    std::vector<Vertex> exits;
    
    if (boundary.empty()) return exits;
    
    // 将连续的边界点分组
    std::vector<std::vector<Vertex>> continuous_groups;
    std::vector<Vertex> current_group = {boundary[0]};
    
    for (size_t i = 1; i < boundary.size(); ++i) {
        if (abs(boundary[i-1].x - boundary[i].x) + abs(boundary[i-1].y - boundary[i].y) == 1) {
            current_group.push_back(boundary[i]);
        } else {
            if (!current_group.empty()) {
                continuous_groups.push_back(current_group);
            }
            current_group = {boundary[i]};
        }
    }
    
    // 处理最后一组
    if (!current_group.empty()) {
        continuous_groups.push_back(current_group);
    }
    
    // 为每个连续组生成出口点
    for (const auto& group : continuous_groups) {
        if (group.size() == 1) {
            // 单个点：生成一个出口点
            exits.push_back(group[0]);
        } else if (group.size() < WIDTH_THRESHOLD) {
            // 宽度 < 阈值：在组中点生成一个出口点
            int mid_index = group.size() / 2;
            exits.push_back(group[mid_index]);
        } else {
            // 宽度 ≥ 阈值：在组两端各生成一个出口点
            exits.push_back(group.front());
            exits.push_back(group.back());
        }
    }
    
    return exits;
}

void HPAStar::compute_abstract_edges() {
    abstract_edges_.clear();
    
    for (const auto& cluster : clusters_) {
        // 计算聚类内所有出口点到出口点的路径（双向）
        for (size_t i = 0; i < cluster.exits.size(); ++i) {
            for (size_t j = 0; j < cluster.exits.size(); ++j) {
                if (i != j) {
                    std::vector<Vertex> path = a_star(cluster.exits[i], cluster.exits[j], grid_);
                    if (!path.empty()) {
                        double cost = path.size() - 1;
                        abstract_edges_[cluster.exits[i]][cluster.exits[j]] = AbstractEdge(cost, path);
                        // 反向边
                        std::vector<Vertex> reverse_path = path;
                        std::reverse(reverse_path.begin(), reverse_path.end());
                        abstract_edges_[cluster.exits[j]][cluster.exits[i]] = AbstractEdge(cost, reverse_path);
                    }
                }
            }
        }
        
        // 计算相邻聚类之间的边（相邻出口点的双向连接）
        for (const auto& exit : cluster.exits) {
            for (const auto& other_cluster : clusters_) {
                if (other_cluster.id != cluster.id) {
                    // 检查出口是否连接到其他聚类的相邻出口
                    for (const auto& other_exit : other_cluster.exits) {
                        // 检查两个出口点是否相邻（曼哈顿距离为1）
                        int dx = abs(exit.x - other_exit.x);
                        int dy = abs(exit.y - other_exit.y);
                        if ((dx == 1 && dy == 0) || (dx == 0 && dy == 1)) {
                            // 两个出口点相邻，添加双向连接边
                            std::vector<Vertex> step_path = {exit, other_exit};
                            abstract_edges_[exit][other_exit] = AbstractEdge(1.0, step_path);
                            std::vector<Vertex> reverse_step_path = {other_exit, exit};
                            abstract_edges_[other_exit][exit] = AbstractEdge(1.0, reverse_step_path);
                            break;
                        }
                    }

                }
            }
        }
    }
}


std::vector<Vertex> HPAStar::search(const Vertex& start, const Vertex& target) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    memory_before_ = memory_utils::get_current_memory_usage();
    expanded_nodes_ = 0;
    
    std::vector<Vertex> result;
    
    // 如果起点和终点在同一聚类内，直接使用A*
    if (same_cluster(start, target)) {
        result = a_star(start, target, grid_);
        // 注意：a_star函数不返回扩展节点数，这里暂时设为0
        expanded_nodes_ = 0;
    } else {
        // 使用抽象图搜索
        result = search_abstract_graph(start, target);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    search_time_ = duration.count() / 1000000.0;
    
    memory_after_ = memory_utils::get_current_memory_usage();
    
    return result;
}

std::vector<Vertex> HPAStar::search_abstract_graph(const Vertex& start, const Vertex& target) {
    std::priority_queue<std::shared_ptr<AbstractNode>, 
                       std::vector<std::shared_ptr<AbstractNode>>, 
                       AbstractNodeComparator> open_list;
    
    std::unordered_map<Vertex, double, VertexHash> g_values;
    
    // 获取起点和终点所在的聚类
    int start_cluster = get_cluster_id(start);
    int target_cluster = get_cluster_id(target);
    
    // 创建起点和终点的临时抽象节点
    auto start_node = std::make_shared<AbstractNode>(start, 0.0, heuristic(start, target));
    auto target_node = std::make_shared<AbstractNode>(target, 0.0, 0.0);
    
    // 记录需要删除的临时边
    std::vector<std::pair<Vertex, Vertex>> temp_edge_keys;
    
    // 将起点连接到其聚类的所有出口点
    for (const auto& cluster : clusters_) {
        if (cluster.id == start_cluster) {
            for (const auto& exit : cluster.exits) {
                // 计算从起点到出口点的路径
                std::vector<Vertex> path = a_star(start, exit, grid_);
                if (!path.empty()) {
                    double cost = path.size() - 1;
                    abstract_edges_[start][exit] = AbstractEdge(cost, path);
                    temp_edge_keys.push_back(std::make_pair(start, exit));
                    
                    auto exit_node = std::make_shared<AbstractNode>(exit, cost, heuristic(exit, target), start_node);
                    open_list.push(exit_node);
                    g_values[exit] = cost;
                }
            }
            break;
        }
    }
    
    // 将终点连接到其聚类的所有出口点
    for (const auto& cluster : clusters_) {
        if (cluster.id == target_cluster) {
            for (const auto& exit : cluster.exits) {
                // 计算从出口点到终点的路径
                std::vector<Vertex> path = a_star(exit, target, grid_);
                if (!path.empty()) {
                    double cost = path.size() - 1;
                    abstract_edges_[exit][target] = AbstractEdge(cost, path);
                    temp_edge_keys.push_back(std::make_pair(exit, target));
                }
            }
            break;
        }
    }
    
    while (!open_list.empty()) {
        auto current = open_list.top();
        open_list.pop();
        expanded_nodes_++;
        
        // 检查是否到达终点
        if (current->exit == target) {
            std::vector<Vertex> path = reconstruct_path(current, start, target);
            for (const auto& key : temp_edge_keys) {
                abstract_edges_[key.first].erase(key.second);
                if (abstract_edges_[key.first].empty()) {
                    abstract_edges_.erase(key.first);
                }
            }
            return path;
        }
        
        // 遍历所有抽象边，找到从当前出口点出发的边
        auto from_it = abstract_edges_.find(current->exit);
        if (from_it != abstract_edges_.end()) {
            for (const auto& to_map : from_it->second) {
                const Vertex& to = to_map.first;
                const AbstractEdge& edge = to_map.second;
                double new_g = current->g_cost + edge.cost;
                if (g_values.find(to) == g_values.end() || new_g < g_values[to]) {
                    g_values[to] = new_g;
                    
                    auto next_node = std::make_shared<AbstractNode>(
                        to, new_g, heuristic(to, target), current
                    );
                    
                    open_list.push(next_node);
                }
            }
        }
    }
    
    // 清理临时边
    for (const auto& key : temp_edge_keys) {
        abstract_edges_[key.first].erase(key.second);
        if (abstract_edges_[key.first].empty()) {
            abstract_edges_.erase(key.first);
        }
    }
    
    return std::vector<Vertex>();
}

std::vector<Vertex> HPAStar::reconstruct_path(const std::shared_ptr<AbstractNode>& goal_node, 
                                              const Vertex& start, const Vertex& target) {
    std::vector<Vertex> path;
    // 从目标节点回溯到起始节点
    std::vector<std::shared_ptr<AbstractNode>> abstract_path;
    auto current = goal_node;
    while (current) {
        abstract_path.push_back(current);
        current = current->parent;
    }
    std::reverse(abstract_path.begin(), abstract_path.end());
    if (abstract_path.empty()) {
        return path;
    }
    // 统一拼接每一段
    for (size_t i = 1; i < abstract_path.size(); ++i) {
        Vertex from = (i == 1) ? start : abstract_path[i-1]->exit;
        Vertex to = abstract_path[i]->exit;
        auto it_outer = abstract_edges_.find(from);
        std::vector<Vertex> segment;
        if (it_outer != abstract_edges_.end()) {
            auto it_inner = it_outer->second.find(to);
            if (it_inner != it_outer->second.end()) {
                segment = it_inner->second.path;
            } else {
                segment = a_star(from, to, grid_);
            }
        } else {
            segment = a_star(from, to, grid_);
        }
        if (!segment.empty()) {
            if (!path.empty() && path.back() == segment.front()) {
                path.insert(path.end(), segment.begin() + 1, segment.end());
            } else {
                path.insert(path.end(), segment.begin(), segment.end());
            }
        }
    }
    return path;
}


int HPAStar::get_cluster_id(const Vertex& v) const {
    for (const auto& cluster : clusters_) {
        if (cluster.contains(v)) {
            return cluster.id;
        }
    }
    return -1;
}

bool HPAStar::same_cluster(const Vertex& a, const Vertex& b) const {
    return get_cluster_id(a) == get_cluster_id(b) && get_cluster_id(a) != -1;
}

size_t HPAStar::getSearchMemoryIncrease() const {
    return memory_after_ - memory_before_;
}

void HPAStar::resetSearchMemoryUsage() {
    memory_before_ = 0;
    memory_after_ = 0;
} 

