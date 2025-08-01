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
    ////logger::log_info("Preprocessing HPAStar");
    auto start_time = std::chrono::high_resolution_clock::now();
    
    clusters_.clear();
    abstract_edges_.clear();
    
    grid_ = grid;

    ////logger::log_info("Creating clusters");
    // 创建聚类
    create_clusters();
    
    ////logger::log_info("Identifying entrances and exits");
    // 识别入口和出口点
    identify_entrances_and_exits();
    
    ////logger::log_info("Computing abstract edges");
    // 计算抽象边
    compute_abstract_edges();
    
    // 直接计算内存使用量，而不是使用系统内存监测
    preprocess_memory_ = getMemoryUsage();
    
    ////logger::log_info("Preprocess memory usage: " + std::to_string(preprocess_memory_));
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    preprocess_time_ = duration.count() / 1000000.0;
    ////logger::log_info("Preprocessing completed");
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
                auto exits = generate_exits_from_boundary(boundary_points);
                for (const auto& exit : exits) {
                    //logger::log_info("[HPAStar] exit: (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ")");
                    cluster.exits.insert(exit);
                    Cluster* right_cluster = find_cluster_by_position(exit.x, exit.y + 1);
                    if (right_cluster) {
                        Vertex right_exit(exit.x, exit.y + 1);
                        right_cluster->exits.insert(right_exit);
                        // 直接添加抽象边
                        abstract_edges_[exit][right_exit] = AbstractEdge(1.0);
                        abstract_edges_[right_exit][exit] = AbstractEdge(1.0);
                        //logger::log_info("[HPAStar] abstract_edges: (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ") -> (" + std::to_string(right_exit.x) + "," + std::to_string(right_exit.y) + ")");
                        //logger::log_info("[HPAStar] abstract_edges: (" + std::to_string(right_exit.x) + "," + std::to_string(right_exit.y) + ") -> (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ")");
                    }
                }
                boundary_points.clear();
            }
        }
        // 处理最后一组
        if (!boundary_points.empty()) {
            auto exits = generate_exits_from_boundary(boundary_points);
            for (const auto& exit : exits) {
                //logger::log_info("[HPAStar] exit: (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ")");
                cluster.exits.insert(exit);
                Cluster* right_cluster = find_cluster_by_position(exit.x, exit.y + 1);
                if (right_cluster) {
                    Vertex right_exit(exit.x, exit.y + 1);
                    right_cluster->exits.insert(right_exit);
                    // 直接添加抽象边
                    abstract_edges_[exit][right_exit] = AbstractEdge(1.0);
                    abstract_edges_[right_exit][exit] = AbstractEdge(1.0);
                    //logger::log_info("[HPAStar] abstract_edges: (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ") -> (" + std::to_string(right_exit.x) + "," + std::to_string(right_exit.y) + ")");
                    //logger::log_info("[HPAStar] abstract_edges: (" + std::to_string(right_exit.x) + "," + std::to_string(right_exit.y) + ") -> (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ")");
                }
            }
            boundary_points.clear();
        }
        // 下边界：固定行，列递增
        boundary_points.clear();
        int bottom_row = cluster.bottom_right.x; // x 实际为 row
        for (int col = cluster.top_left.y; col <= cluster.bottom_right.y; ++col) { // y 实际为 col
            Vertex v(bottom_row, col);
            if (utils::isPassable(grid_, v) && utils::isPassable(grid_, Vertex(bottom_row + 1, col))) {
                boundary_points.push_back(v);
            } else if (!boundary_points.empty()) {
                auto exits = generate_exits_from_boundary(boundary_points);
                for (const auto& exit : exits) {
                    //logger::log_info("[HPAStar] exit: (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ")");
                    cluster.exits.insert(exit);
                    Cluster* bottom_cluster = find_cluster_by_position(exit.x + 1, exit.y);
                    if (bottom_cluster) {
                        Vertex bottom_exit(exit.x + 1, exit.y);
                        bottom_cluster->exits.insert(bottom_exit);
                        // 直接添加抽象边
                        abstract_edges_[exit][bottom_exit] = AbstractEdge(1.0);
                        abstract_edges_[bottom_exit][exit] = AbstractEdge(1.0);
                        //logger::log_info("[HPAStar] abstract_edges: (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ") -> (" + std::to_string(bottom_exit.x) + "," + std::to_string(bottom_exit.y) + ")");
                        //logger::log_info("[HPAStar] abstract_edges: (" + std::to_string(bottom_exit.x) + "," + std::to_string(bottom_exit.y) + ") -> (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ")");
                    }
                }
                boundary_points.clear();
            }
        }
        if (!boundary_points.empty()) {
            auto exits = generate_exits_from_boundary(boundary_points);
            for (const auto& exit : exits) {
                //logger::log_info("[HPAStar] exit: (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ")");
                cluster.exits.insert(exit);
                Cluster* bottom_cluster = find_cluster_by_position(exit.x + 1, exit.y);
                if (bottom_cluster) {
                    Vertex bottom_exit(exit.x + 1, exit.y);
                    bottom_cluster->exits.insert(bottom_exit);
                    // 直接添加抽象边
                    abstract_edges_[exit][bottom_exit] = AbstractEdge(1.0);
                    abstract_edges_[bottom_exit][exit] = AbstractEdge(1.0);
                    //logger::log_info("[HPAStar] abstract_edges: (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ") -> (" + std::to_string(bottom_exit.x) + "," + std::to_string(bottom_exit.y) + ")");
                    //logger::log_info("[HPAStar] abstract_edges: (" + std::to_string(bottom_exit.x) + "," + std::to_string(bottom_exit.y) + ") -> (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ")");
                }
            }
            boundary_points.clear();
        }
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
    // 直接对boundary整体生成出口点
    if (boundary.size() == 1) {
        exits.push_back(boundary[0]);
    } else if (boundary.size() < WIDTH_THRESHOLD) {
        int mid_index = boundary.size() / 2;
        exits.push_back(boundary[mid_index]);
    } else {
        exits.push_back(boundary.front());
        exits.push_back(boundary.back());
    }
    return exits;
}

void HPAStar::compute_abstract_edges() {
    
    for (const auto& cluster : clusters_) {
        // compute_abstract_edges只保留聚类内出口点之间的最短路
        std::vector<Vertex> exits_vec(cluster.exits.begin(), cluster.exits.end());
        for (size_t i = 0; i < exits_vec.size(); ++i) {
            for (size_t j = i + 1; j < exits_vec.size(); ++j) {
                const Vertex& a = exits_vec[i];
                const Vertex& b = exits_vec[j];
                // 使用边界限制的A*搜索，限制在cluster范围内
                std::vector<Vertex> path = a_star(a, b, grid_, cluster.top_left, cluster.bottom_right);
                if (!path.empty()) {
                    double cost = path.size() - 1;
                    abstract_edges_[a][b] = AbstractEdge(cost);
                    abstract_edges_[b][a] = AbstractEdge(cost);
                }
            }
        }
    }
}


std::vector<Vertex> HPAStar::search(const Vertex& start, const Vertex& target) {
    //logger::log_info("[HPAStar] start searching from (" + std::to_string(start.x) + "," + std::to_string(start.y) + ") to (" + std::to_string(target.x) + "," + std::to_string(target.y) + ")");
    auto start_time = std::chrono::high_resolution_clock::now();
    
    memory_before_ = memory_utils::get_current_memory_usage();
    expanded_nodes_ = 0;
    
    std::vector<Vertex> result;
    
    // 如果起点和终点在同一聚类内，直接使用A*
    if (same_cluster(start, target)) {
        //logger::log_info("[HPAStar] start searching in same cluster");
        // 找到对应的cluster，使用边界限制的A*
        int cluster_id = get_cluster_id(start);
        for (const auto& cluster : clusters_) {
            if (cluster.id == cluster_id) {
                result = a_star(start, target, grid_);
                break;
            }
        }
        // 注意：a_star函数不返回扩展节点数，这里暂时设为0
        expanded_nodes_ = 0;
    } else {
        //logger::log_info("[HPAStar] start searching in different clusters");
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
    //logger::log_info("[HPAStar] start searching in abstract graph");
    std::priority_queue<std::shared_ptr<AbstractNode>, 
                       std::vector<std::shared_ptr<AbstractNode>>, 
                       AbstractNodeComparator> open_list;
    
    std::unordered_map<Vertex, double> g_values;
    
    // 获取起点和终点所在的聚类
    int start_cluster = get_cluster_id(start);
    int target_cluster = get_cluster_id(target);
    //logger::log_info("[HPAStar] start_cluster: " + std::to_string(start_cluster) + ", target_cluster: " + std::to_string(target_cluster));

    // 1. 起点到出口点
    for (const auto& cluster : clusters_) {
        if (cluster.id == start_cluster) {
            auto start_node = std::make_shared<AbstractNode>(start, 0.0, heuristic(start, target));
            if(cluster.exits.find(start) != cluster.exits.end()) {
                open_list.push(start_node);
                g_values[start] = 0.0;
                break;
            }
            for (const auto& exit : cluster.exits) {
                // 使用边界限制的A*搜索，限制在start cluster范围内
                std::vector<Vertex> path = a_star(start, exit, grid_, cluster.top_left, cluster.bottom_right);
                if (!path.empty()) {
                    double cost = path.size() - 1;
                    auto exit_node = std::make_shared<AbstractNode>(exit, cost, heuristic(exit, target), start_node);
                    open_list.push(exit_node);
                    g_values[exit] = cost;
                    //logger::log_info("[HPAStar] start_edges: (" + std::to_string(start.x) + "," + std::to_string(start.y) + ") -> (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ")");
                }
            }
            break;
        }
    }

    // 2. 终点相关
    std::unordered_map<Vertex, AbstractEdge> target_edges;
    for (const auto& cluster : clusters_) {
        if (cluster.id == target_cluster) {
            if(cluster.exits.find(target) != cluster.exits.end()) {
                break;
            }
            for (const auto& exit : cluster.exits) {
                // 使用边界限制的A*搜索，限制在target cluster范围内
                std::vector<Vertex> path = a_star(exit, target, grid_, cluster.top_left, cluster.bottom_right);
                if (!path.empty()) {
                    double cost = path.size() - 1;
                    target_edges[exit] = AbstractEdge(cost);
                    //logger::log_info("[HPAStar] target_edges: (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ")");
                }
            }
            break;
        }
    }

    // 3. 搜索
    while (!open_list.empty()) {
        auto current = open_list.top();
        open_list.pop();
        expanded_nodes_++;
        //logger::log_info("[HPAStar] current node: (" + std::to_string(current->exit.x) + "," + std::to_string(current->exit.y) + ")");
        
        if(current->exit == target) {
            std::vector<Vertex> path = reconstruct_path(current, start, target);
            return path;
        }

        // 检查是否能直接到达 target
        auto it_target = target_edges.find(current->exit);
        if (it_target != target_edges.end()) {
            double new_g = current->g_cost + it_target->second.cost;
            auto target_node = std::make_shared<AbstractNode>(target, new_g, 0.0, current);
            
            if(g_values.find(target) == g_values.end() || new_g < g_values[target]) {
                g_values[target] = new_g;
                open_list.push(target_node);
            }
        }

        // 遍历所有抽象边，找到从当前出口点出发的边
        auto from_it = abstract_edges_.find(current->exit);
        if (from_it != abstract_edges_.end()) {
            for (const auto& to_map : from_it->second) {
                //logger::log_info("[HPAStar] expand abstract_edges: (" + std::to_string(current->exit.x) + "," + std::to_string(current->exit.y) + ") -> (" + std::to_string(to_map.first.x) + "," + std::to_string(to_map.first.y) + ")");
                const Vertex& to = to_map.first;
                const AbstractEdge& edge = to_map.second;
                double new_g = current->g_cost + edge.cost;

                // if (g_values.find(to) != g_values.end()) {
                //     //logger::log_info("[HPAStar] has already g_value: (" + std::to_string(g_values[to]) + ")");
                //     if (new_g >= g_values[to]) {
                //         //logger::log_info("[HPAStar] node with higher g_cost: (" + std::to_string(to.x) + "," + std::to_string(to.y) + ")");
                //     }
                // }

                if (g_values.find(to) == g_values.end() || new_g < g_values[to]) {
                    g_values[to] = new_g;
                    auto next_node = std::make_shared<AbstractNode>(to, new_g, heuristic(to, target), current);
                    open_list.push(next_node);
                    //logger::log_info("[HPAStar] push node: (" + std::to_string(to.x) + "," + std::to_string(to.y) + ")");
                }
            }
        }
    }
    //logger::log_info("[HPAStar] open_list is empty, search failed");
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
    for (size_t i = 1; i < abstract_path.size(); ++i) {
        Vertex from = abstract_path[i-1]->exit;
        Vertex to = abstract_path[i]->exit;
        std::vector<Vertex> segment = a_star(from, to, grid_);
        if (segment.empty()) {
            //logger::log_error("[HPAStar] connect failed: from (" + std::to_string(from.x) + "," + std::to_string(from.y) + ") to (" + std::to_string(to.x) + "," + std::to_string(to.y) + ")");
            //logger::log_error("[HPAStar] abstract_path:");
            for (const auto& node : abstract_path) {
                //logger::log_error("  (" + std::to_string(node->exit.x) + "," + std::to_string(node->exit.y) + ")");
            }
            return {};
        }
        if (i == 1) {
            path = segment;
        } else {
            path.insert(path.end(), segment.begin() + 1, segment.end());
        }
        //logger::log_info("[HPAStar]connect from (" + std::to_string(from.x) + "," + std::to_string(from.y) + ") to (" + std::to_string(to.x) + "," + std::to_string(to.y) + ")");
    }
    //logger::log_info("[HPAStar] path length: " + std::to_string(path.size()));
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

size_t HPAStar::getMemoryUsage() const {
    size_t total_memory = 0;
    
    // 计算grid_占用的内存
    if (!grid_.empty() && !grid_[0].empty()) {
        total_memory += grid_.size() * grid_[0].size() * sizeof(int);
    }
    
    // 计算clusters_占用的内存
    for (const auto& cluster : clusters_) {
        // Cluster结构体大小
        total_memory += sizeof(Cluster);
        // exits集合占用的内存
        total_memory += cluster.exits.size() * sizeof(Vertex);
    }
    
    // 计算abstract_edges_占用的内存
    for (const auto& [from_vertex, edge_map] : abstract_edges_) {
        // 外层map的key (Vertex)
        total_memory += sizeof(Vertex);
        // 内层map的大小
        total_memory += edge_map.size() * sizeof(std::pair<Vertex, AbstractEdge>);
    }
    
    return total_memory;
} 

