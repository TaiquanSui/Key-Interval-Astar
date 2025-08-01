#ifndef HPASTAR_H
#define HPASTAR_H

#include "../basic/Vertex.h"
#include "../solver/SolverInterface.h"
#include "../utilities/MemoryUtility.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <queue>
#include <string>

// abstract node structure
struct AbstractNode {
    Vertex exit;              // exit point
    double g_cost;            // cost from start to this node
    double h_cost;            // heuristic cost
    std::shared_ptr<AbstractNode> parent;
    
    AbstractNode(Vertex ext, double g = 0, double h = 0, 
                 std::shared_ptr<AbstractNode> p = nullptr)
        : exit(ext), g_cost(g), h_cost(h), parent(p) {}
    
    double f() const { return g_cost + h_cost; }
};

// abstract node comparator
struct AbstractNodeComparator {
    bool operator()(const std::shared_ptr<AbstractNode>& a, 
                   const std::shared_ptr<AbstractNode>& b) const {
        if (std::abs(a->f() - b->f()) > 1e-6) {
            return a->f() > b->f();
        }
        return a->g_cost < b->g_cost;
    }
};

// cluster structure
struct Cluster {
    int id;
    Vertex top_left;          // top left coordinate
    Vertex bottom_right;      // bottom right coordinate
    std::unordered_set<Vertex> exits;  // exit point list (simplified)
    
    Cluster(int cluster_id, Vertex tl, Vertex br) 
        : id(cluster_id), top_left(tl), bottom_right(br), exits() {}
    
    bool contains(const Vertex& v) const {
        return v.x >= top_left.x && v.x <= bottom_right.x &&
               v.y >= top_left.y && v.y <= bottom_right.y;
    }
    
    int width() const { return bottom_right.x - top_left.x + 1; }
    int height() const { return bottom_right.y - top_left.y + 1; }
};

// abstract edge structure
struct AbstractEdge {
    double cost;
    AbstractEdge(double c) : cost(c) {}
    AbstractEdge() : cost(0.0) {}
};


// HPA* algorithm class
class HPAStar : public SolverInterface {
private:
    std::vector<std::vector<int>> grid_;
    std::vector<Cluster> clusters_;
    std::unordered_map<Vertex, std::unordered_map<Vertex, AbstractEdge>> abstract_edges_;
    int cluster_size_;
    int expanded_nodes_;
    double search_time_;
    double preprocess_time_;
    std::string name_;
    size_t memory_before_;
    size_t memory_after_;
    size_t preprocess_memory_;

public:
    HPAStar(const std::string& name = "HPAStar", int cluster_size = 10) 
        : cluster_size_(cluster_size), expanded_nodes_(0), search_time_(0.0), 
          preprocess_time_(0.0), name_(name), memory_before_(0), memory_after_(0), 
          preprocess_memory_(0) {}
    
    // implement SolverInterface interface
    std::vector<Vertex> search(const Vertex& start, const Vertex& target) override;
    std::string get_name() const override { return name_; }
    int getExpandedNodes() const override { return expanded_nodes_; }
    void resetExpandedNodes() override { expanded_nodes_ = 0; }
    double getSearchTime() const override { return search_time_; }
    void resetSearchTime() override { search_time_ = 0.0; }
    
    // memory monitoring interface implementation
    size_t getPreprocessMemoryUsage() const override { return preprocess_memory_; }
    size_t getSearchMemoryIncrease() const override;
    void resetSearchMemoryUsage() override;
    double getPreprocessTime() const override { return preprocess_time_; }
    
    // directly calculate memory usage
    size_t getMemoryUsage() const;
    
    // preprocessing interface
    void preprocess(const std::vector<std::vector<int>>& grid) override;

private:
    // create clusters
    void create_clusters();
    
    // identify entrances and exits
    void identify_entrances_and_exits();
    
    // compute abstract edges
    void compute_abstract_edges();
    
    // search on abstract graph
    std::vector<Vertex> search_abstract_graph(const Vertex& start, const Vertex& target);
    
    // reconstruct full path
    std::vector<Vertex> reconstruct_path(const std::shared_ptr<AbstractNode>& goal_node, 
                                        const Vertex& start, const Vertex& target);
    
    // generate exit points
    std::vector<Vertex> generate_exits_from_boundary(const std::vector<Vertex>& boundary) const;
    
    // get cluster ID containing specified point
    int get_cluster_id(const Vertex& v) const;
    
    // check if two points are in the same cluster
    bool same_cluster(const Vertex& a, const Vertex& b) const;

    // auxiliary: find cluster pointer by position
    Cluster* find_cluster_by_position(int x, int y);
};

#endif // HPASTAR_H 