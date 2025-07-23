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
        // 1. f(n) 低的优先
        if (std::abs(a->f() - b->f()) > 1e-6) {
            return a->f() > b->f();
        }

        // g(n) 高的优先
        if (std::abs(a->g - b->g) > 1e-6) {
            return a->g < b->g;  // 注意这里是 < 因为我们要高的优先
        }

        // 时间戳早的优先
        return a->time > b->time;
    }
};

// A*搜索器类，直接实现SolverInterface接口
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
    
    // 新增：默认构造函数，用于延迟初始化
    AStar(const std::string& name = "AStar") : name_(name) {}
    
    // 实现SolverInterface接口
    std::vector<Vertex> search(const Vertex& start, const Vertex& target) override;
    std::string get_name() const override { return name_; }
    int getExpandedNodes() const override { return expanded_nodes_; }
    void resetExpandedNodes() override { expanded_nodes_ = 0; }
    double getSearchTime() const override { return search_time_; }
    void resetSearchTime() override { search_time_ = 0.0; }

    // 内存监控接口实现
    size_t getPreprocessMemoryUsage() const override { return 0; }
    size_t getSearchMemoryIncrease() const override;
    void resetSearchMemoryUsage() override;
    double getPreprocessTime() const override { return 0.0; }
    
    // 实现预处理接口（A*不需要预处理，但需要更新地图）
    void preprocess(const std::vector<std::vector<int>>& grid) override { grid_ = grid; }

private:
    std::vector<Vertex> reconstruct_path(const std::shared_ptr<AStarNode>& goal_node) const;
};

// Basic A* search (保持向后兼容)
std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal, 
                          const std::vector<std::vector<int>>& grid);

// 支持自定义方向的A*搜索
std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal,
                          const std::vector<std::vector<int>>& grid,
                          const std::vector<Vertex>& directions);

#endif // ASTAR_H