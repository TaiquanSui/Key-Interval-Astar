#pragma once

#include <vector>
#include <queue>
#include <unordered_map>
#include <set>
#include "../basic/Vertex.h"
#include "Preprocess2.h"

class HighLevelSearch {
public:
    // 定义子图节点
    struct SubgraphNode {
        size_t subspaceIndex;  // 对应的子空间索引
        Vertex entrance;       // 入口点
        Vertex exit;          // 出口点
        double g;             // 从起点到当前节点的代价
        double h;             // 从当前节点到终点的启发式估计
        double f;             // f = g + h
        size_t parent;        // 父节点索引
        bool closed;          // 是否已关闭

        SubgraphNode(size_t idx, const Vertex& ent, const Vertex& ex, double g_val, double h_val)
            : subspaceIndex(idx), entrance(ent), exit(ex), 
              g(g_val), h(h_val), f(g_val + h_val), 
              parent(0), closed(false) {}
    };

    // 构造函数
    HighLevelSearch(const std::vector<Preprocess2::Subspace>& subspaces);

    // 在高层图上搜索路径
    std::vector<Vertex> search(const Vertex& start, const Vertex& goal);

private:
    // 子空间集合
    const std::vector<Preprocess2::Subspace>& subspaces_;

    // 找到包含给定点的子空间索引
    size_t findSubspaceContainingPoint(const Vertex& point) const;

    // 计算两个点之间的启发式距离
    double calculateHeuristic(const Vertex& a, const Vertex& b) const;

    // 计算两个子空间之间的代价
    double calculateCost(const SubgraphNode& from, const SubgraphNode& to) const;

    // 获取子空间的入口和出口点
    void getEntranceAndExit(const Preprocess2::Subspace& subspace, 
                           const Vertex& from, const Vertex& to,
                           Vertex& entrance, Vertex& exit) const;

    // 从关键点集合中选择最佳入口和出口
    void selectBestPoints(const std::set<Preprocess2::KeyPoint>& keyPoints,
                         const Vertex& from, const Vertex& to,
                         Vertex& entrance, Vertex& exit) const;
}; 