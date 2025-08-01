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
    // create clusters
    create_clusters();
    
    ////logger::log_info("Identifying entrances and exits");
    // identify entrances and exits
    identify_entrances_and_exits();
    
    ////logger::log_info("Computing abstract edges");
    // compute abstract edges
    compute_abstract_edges();
    
    // directly calculate memory usage, instead of using system memory monitoring
    preprocess_memory_ = getMemoryUsage();
    
    ////logger::log_info("Preprocess memory usage: " + std::to_string(preprocess_memory_));
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    preprocess_time_ = duration.count() / 1000000.0;
    ////logger::log_info("Preprocessing completed");
}

void HPAStar::create_clusters() {
    if (grid_.empty() || grid_[0].empty()) return;
    
    int rows = grid_.size();    // row number
    int cols = grid_[0].size(); // column number
    int cluster_id = 0;
    // split by row (row) and column (col)
    for (int row = 0; row < rows; row += cluster_size_) {
        for (int col = 0; col < cols; col += cluster_size_) {
            Vertex top_left(row, col); // x=row, y=col
            Vertex bottom_right(std::min(row + cluster_size_ - 1, rows - 1), 
                               std::min(col + cluster_size_ - 1, cols - 1));
            // only create clusters containing traversable points
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
    int rows = grid_.size();    // row number
    int cols = grid_[0].size(); // column number
    // only process right boundary and bottom boundary
    for (auto& cluster : clusters_) {
        // right boundary: fixed column, row increment
        std::vector<Vertex> boundary_points;
        int right_col = cluster.bottom_right.y; // y is actually col
        for (int row = cluster.top_left.x; row <= cluster.bottom_right.x; ++row) { // x is actually row
            Vertex v(row, right_col); // Vertex(x=row, y=col)
            // check if the point is traversable and the next point is also traversable
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
                        // directly add abstract edge
                        abstract_edges_[exit][right_exit] = AbstractEdge(1.0);
                        abstract_edges_[right_exit][exit] = AbstractEdge(1.0);
                        //logger::log_info("[HPAStar] abstract_edges: (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ") -> (" + std::to_string(right_exit.x) + "," + std::to_string(right_exit.y) + ")");
                        //logger::log_info("[HPAStar] abstract_edges: (" + std::to_string(right_exit.x) + "," + std::to_string(right_exit.y) + ") -> (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ")");
                    }
                }
                boundary_points.clear();
            }
        }
        // process the last group
        if (!boundary_points.empty()) {
            auto exits = generate_exits_from_boundary(boundary_points);
            for (const auto& exit : exits) {
                //logger::log_info("[HPAStar] exit: (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ")");
                cluster.exits.insert(exit);
                Cluster* right_cluster = find_cluster_by_position(exit.x, exit.y + 1);
                if (right_cluster) {
                    Vertex right_exit(exit.x, exit.y + 1);
                    right_cluster->exits.insert(right_exit);
                    // directly add abstract edge
                    abstract_edges_[exit][right_exit] = AbstractEdge(1.0);
                    abstract_edges_[right_exit][exit] = AbstractEdge(1.0);
                    //logger::log_info("[HPAStar] abstract_edges: (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ") -> (" + std::to_string(right_exit.x) + "," + std::to_string(right_exit.y) + ")");
                    //logger::log_info("[HPAStar] abstract_edges: (" + std::to_string(right_exit.x) + "," + std::to_string(right_exit.y) + ") -> (" + std::to_string(exit.x) + "," + std::to_string(exit.y) + ")");
                }
            }
            boundary_points.clear();
        }
        // bottom boundary: fixed row, column increment
        boundary_points.clear();
        int bottom_row = cluster.bottom_right.x; // x is actually row
        for (int col = cluster.top_left.y; col <= cluster.bottom_right.y; ++col) { // y is actually col
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
                        // directly add abstract edge
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

// auxiliary: find cluster pointer by position
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
    // directly generate exits from boundary
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
        // compute_abstract_edges only keeps the shortest path between exits in the cluster
        std::vector<Vertex> exits_vec(cluster.exits.begin(), cluster.exits.end());
        for (size_t i = 0; i < exits_vec.size(); ++i) {
            for (size_t j = i + 1; j < exits_vec.size(); ++j) {
                const Vertex& a = exits_vec[i];
                const Vertex& b = exits_vec[j];
                // use boundary restricted A* search, limited to cluster range
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
    
    // if start and target are in the same cluster, use A* directly
    if (same_cluster(start, target)) {
        //logger::log_info("[HPAStar] start searching in same cluster");
        // find the corresponding cluster, use boundary restricted A*
        int cluster_id = get_cluster_id(start);
        for (const auto& cluster : clusters_) {
            if (cluster.id == cluster_id) {
                result = a_star(start, target, grid_);
                break;
            }
        }
        // note: a_star function does not return expanded node count, here temporarily set to 0
        expanded_nodes_ = 0;
    } else {
        //logger::log_info("[HPAStar] start searching in different clusters");
        // use abstract graph search
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
    
    // get the cluster where the start and target are located
    int start_cluster = get_cluster_id(start);
    int target_cluster = get_cluster_id(target);
    //logger::log_info("[HPAStar] start_cluster: " + std::to_string(start_cluster) + ", target_cluster: " + std::to_string(target_cluster));

    // 1. start to exit
    for (const auto& cluster : clusters_) {
        if (cluster.id == start_cluster) {
            auto start_node = std::make_shared<AbstractNode>(start, 0.0, heuristic(start, target));
            if(cluster.exits.find(start) != cluster.exits.end()) {
                open_list.push(start_node);
                g_values[start] = 0.0;
                break;
            }
            for (const auto& exit : cluster.exits) {
                // use boundary restricted A* search, limited to start cluster range
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

    // 2. target related
    std::unordered_map<Vertex, AbstractEdge> target_edges;
    for (const auto& cluster : clusters_) {
        if (cluster.id == target_cluster) {
            if(cluster.exits.find(target) != cluster.exits.end()) {
                break;
            }
            for (const auto& exit : cluster.exits) {
                // use boundary restricted A* search, limited to target cluster range
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

    // 3. search
    while (!open_list.empty()) {
        auto current = open_list.top();
        open_list.pop();
        expanded_nodes_++;
        //logger::log_info("[HPAStar] current node: (" + std::to_string(current->exit.x) + "," + std::to_string(current->exit.y) + ")");
        
        if(current->exit == target) {
            std::vector<Vertex> path = reconstruct_path(current, start, target);
            return path;
        }

        // check if can directly reach target
        auto it_target = target_edges.find(current->exit);
        if (it_target != target_edges.end()) {
            double new_g = current->g_cost + it_target->second.cost;
            auto target_node = std::make_shared<AbstractNode>(target, new_g, 0.0, current);
            
            if(g_values.find(target) == g_values.end() || new_g < g_values[target]) {
                g_values[target] = new_g;
                open_list.push(target_node);
            }
        }

        // traverse all abstract edges, find edges from current exit point
        auto from_it = abstract_edges_.find(current->exit);
        if (from_it != abstract_edges_.end()) {
            for (const auto& to_map : from_it->second) {
                //logger::log_info("[HPAStar] expand abstract_edges: (" + std::to_string(current->exit.x) + "," + std::to_string(current->exit.y) + ") -> (" + std::to_string(to_map.first.x) + "," + std::to_string(to_map.first.y) + ")");
                const Vertex& to = to_map.first;
                const AbstractEdge& edge = to_map.second;
                double new_g = current->g_cost + edge.cost;

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
    // backtrack from target node to start node
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
    
    // calculate memory usage of grid_
    if (!grid_.empty() && !grid_[0].empty()) {
        total_memory += grid_.size() * grid_[0].size() * sizeof(int);
    }
    
    // calculate memory usage of clusters_
    for (const auto& cluster : clusters_) {
        // Cluster structure size
        total_memory += sizeof(Cluster);
        // memory usage of exits set
        total_memory += cluster.exits.size() * sizeof(Vertex);
    }
    
    // calculate memory usage of abstract_edges_
    for (const auto& [from_vertex, edge_map] : abstract_edges_) {
        // outer map key (Vertex)
        total_memory += sizeof(Vertex);
        // inner map size
        total_memory += edge_map.size() * sizeof(std::pair<Vertex, AbstractEdge>);
    }
    
    return total_memory;
} 

