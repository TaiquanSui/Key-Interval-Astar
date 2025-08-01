#ifndef ASTAR_H
#define ASTAR_H

#include "../basic/Vertex.h"
#include "../solver/SolverInterface.h"
#include <memory>
#include "../utilities/MemoryUtility.h"

struct AStarNode {
    Vertex pos;
    double g;
    double h;
    std::shared_ptr<AStarNode> parent;
    int time;

    AStarNode(Vertex p, double g, double h, std::shared_ptr<AStarNode> parent = nullptr, int t = 0)
        : pos(p), g(g), h(h), parent(std::move(parent)), time(t) {}

    double f() const { return g + h; }
};

struct AStarNodeComparator {
    bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
        // 1. f(n) lower priority
        if (std::abs(a->f() - b->f()) > 1e-6) {
            return a->f() > b->f();
        }

        // g(n) higher priority
        if (std::abs(a->g - b->g) > 1e-6) {
            return a->g < b->g;  // note: here is < because we want higher priority
        }

        // earlier timestamp priority
        return a->time > b->time;
    }
};

// A* search class, directly implement SolverInterface interface
class AStar : public SolverInterface {
private:
    std::vector<std::vector<int>> grid_;
    int expanded_nodes_ = 0;
    double search_time_ = 0.0;
    std::string name_;
    size_t memory_before_ = 0;
    size_t memory_after_ = 0;

public:
    AStar(const std::vector<std::vector<int>>& grid, const std::string& name = "AStar") 
        : grid_(grid), name_(name) {}
    
    // new: default constructor, for delayed initialization
    AStar(const std::string& name = "AStar") : name_(name) {}
    
    // implement SolverInterface interface
    std::vector<Vertex> search(const Vertex& start, const Vertex& target) override;
    std::string get_name() const override { return name_; }
    int getExpandedNodes() const override { return expanded_nodes_; }
    void resetExpandedNodes() override { expanded_nodes_ = 0; }
    double getSearchTime() const override { return search_time_; }
    void resetSearchTime() override { search_time_ = 0.0; }

    // memory monitoring interface implementation
    size_t getPreprocessMemoryUsage() const override { return 0; }
    size_t getSearchMemoryIncrease() const override;
    void resetSearchMemoryUsage() override;
    double getPreprocessTime() const override { return 0.0; }
    
    // implement preprocessing interface (A* does not need preprocessing, but needs to update map)
    void preprocess(const std::vector<std::vector<int>>& grid) override { grid_ = grid; }

private:
    std::vector<Vertex> reconstruct_path(const std::shared_ptr<AStarNode>& goal_node) const;
};

// Basic A* search
std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal, 
                          const std::vector<std::vector<int>>& grid);

// A* search with custom directions
std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal,
                          const std::vector<std::vector<int>>& grid,
                          const std::vector<Vertex>& directions);

// A* search with boundary restriction
std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal,
                          const std::vector<std::vector<int>>& grid,
                          const Vertex& top_left, const Vertex& bottom_right);

#endif // ASTAR_H