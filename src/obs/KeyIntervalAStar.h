#pragma once

#include <vector>
#include <set>
#include <queue>
#include <unordered_map>
#include "../basic/Vertex.h"
#include "Preprocess.h"

class KeyIntervalAStar {
public:
    // 定义搜索节点结构
    struct SearchNode {
        Preprocess::IntervalKey keyInterval;  // key interval的key
        double g;                 // 从起点到当前节点的代价
        double f;                 // 估计的总代价
        Preprocess::IntervalKey parent;  // 父节点的key
        Vertex upVertex;          // 记录y值最小的direction为up的key point
        Vertex downVertex;        // 记录y值最大的direction为up的key point
        std::vector<Vertex> mustPassPoints;  // 必须经过的点集合
        bool isTarget;              // 是否到达终点

        SearchNode(const Preprocess::IntervalKey& key, double g_val, double f_val, 
                  const Preprocess::IntervalKey& p,
                  const Vertex& up, const Vertex& down,
                  const std::vector<Vertex>& mustPass,
                  bool is_end = false)
            : keyInterval(key), g(g_val), f(f_val), parent(p),
              upVertex(up), downVertex(down),
              mustPassPoints(mustPass), isTarget(is_end) {}

        // 用于优先队列的比较
        bool operator>(const SearchNode& other) const {
            return f > other.f;
        }
    };

    // 构造函数
    KeyIntervalAStar(const Preprocess& preprocess);

    // 执行搜索
    std::vector<Vertex> search(const Vertex& start, const Vertex& end);

private:
    const Preprocess& preprocess_;  // 预处理结果

    // 计算两点之间的启发式距离
    double calculateHeuristicDistance(const Vertex& v1, const Vertex& v2) const {
        return std::abs(v1.x - v2.x) + std::abs(v1.y - v2.y);
    }

    // 从指定顶点开始查找最近的key intervals
    std::set<Preprocess::IntervalKey> findNearestKeyIntervals(const Vertex& vertex, bool isStart);

    // 递归查找key intervals
    void findKeyIntervalsRecursively(const Vertex& vertex,
                                   int y,
                                   int x,
                                   std::vector<bool>& visited,
                                   std::set<Preprocess::IntervalKey>& keyIntervals,
                                   bool isStart);

    // 检查vertex是否在某个key interval中
    bool isVertexInKeyInterval(const Vertex& vertex, const Preprocess::IntervalKey& key) const;

    // 计算g值和h值
    std::pair<double, double> calcGAndH(const std::vector<Vertex>& mustPassPoints,
                                      const Preprocess::IntervalKey& keyInterval,
                                      const Vertex& end) const;

    // 构建最终路径
    std::vector<Vertex> buildPath(const SearchNode& targetNode) const;

    // 处理终点key interval
    void handleTargetKeyInterval(const SearchNode& current,
                               const Vertex& target,
                               std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>>& openList);

    // 处理邻居节点
    void handleNeighbor(const SearchNode& current,
                       const IntervalKey& neighborKey,
                       const Vertex& target,
                       std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>>& openList,
                       std::unordered_map<IntervalKey, double>& gScore);

    // 处理mustpasspoint
    void handleMustPassPoint(const SearchNode& current,
                           const IntervalKey& neighborKey,
                           const Vertex& mustPassPoint,
                           const Vertex& target,
                           std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>>& openList,
                           std::unordered_map<IntervalKey, double>& gScore);

    // 处理key points
    void handleKeyPoints(const SearchNode& current,
                        const IntervalKey& neighborKey,
                        const Preprocess::KeyInterval& neighborInterval,
                        const Vertex& target,
                        std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>>& openList,
                        std::unordered_map<IntervalKey, double>& gScore);

}; 