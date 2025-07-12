#ifndef ASTAR_H
#define ASTAR_H

#include "../basic/Vertex.h"
#include <memory>

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
        // if (std::abs(a->g - b->g) > 1e-6) {
        //     return a->g < b->g;  // 注意这里是 < 因为我们要高的优先
        // }

        // 6. 时间戳早的优先
        return a->time > b->time;
    }
};

// Basic A* search
std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal, 
                          const std::vector<std::vector<int>>& grid);

// 支持自定义方向的A*搜索
std::vector<Vertex> a_star(const Vertex& start, const Vertex& goal,
                          const std::vector<std::vector<int>>& grid,
                          const std::vector<Vertex>& directions);

#endif // ASTAR_H