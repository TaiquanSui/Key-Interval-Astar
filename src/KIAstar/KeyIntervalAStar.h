#pragma once

#include <vector>
#include <set>
#include <queue>
#include <unordered_map>
#include <optional>
#include "../basic/Vertex.h"
#include "../utilities/GridUtility.h"
#include "Preprocess.h"

using IntervalKey = Preprocess::IntervalKey;
using KeyInterval = Preprocess::KeyInterval;
using IntervalKeyHash = Preprocess::IntervalKeyHash;
using VerticalInterval = Preprocess::VerticalInterval;
using NeighborTriple = Preprocess::NeighborTriple;

class KeyIntervalAStar {
public:
    // 定义搜索节点结构
    struct SearchNode {
        IntervalKey keyInterval;  // key interval的key
        double g;                 // 从起点到当前节点的代价
        double f;                 // 估计的总代价
        IntervalKey parent;  // 父节点的key
        IntervalKey fromDirectNeighbor;  // 当前直接邻居的key
        Vertex upVertex;          // 记录y值最小的direction为up的key point
        Vertex downVertex;        // 记录y值最大的direction为up的key point
        std::vector<Vertex> waypoints;  // 必须经过的点集合
        bool isTarget;              // 是否到达终点

        SearchNode(const IntervalKey& key, double g_val, double f_val, 
                  const IntervalKey& p,
                  const IntervalKey& fromDirectNeighbor,
                  const Vertex& up, const Vertex& down,
                  const std::vector<Vertex>& waypoints,
                  bool is_end = false)
            : keyInterval(key), g(g_val), f(f_val), parent(p),
              fromDirectNeighbor(fromDirectNeighbor),
              upVertex(up), downVertex(down),
              waypoints(waypoints), isTarget(is_end) {}

        // 用于优先队列的比较
        bool operator>(const SearchNode& other) const {
            return f > other.f;
        }
    };

    enum class ReachabilityResult {
        UNREACHABLE,
        DIRECT_PATH,
        DONE
    };
    
    struct NearestKeyIntervalsResult {
        ReachabilityResult result;
        std::unordered_map<IntervalKey, IntervalKey, IntervalKeyHash> startIntervals;  // key: key interval, value: direct neighbor
        std::unordered_map<IntervalKey, IntervalKey, IntervalKeyHash> targetIntervals; // key: key interval, value: direct neighbor
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

    // 查找起点和终点的key intervals
    NearestKeyIntervalsResult findNearestKeyIntervals(const Vertex& start, const Vertex& target);

    // 查找包含vertex的vertical interval
    std::optional<VerticalInterval> findContainingVerticalInterval(const Vertex& vertex) const;

    // 处理邻居链
    std::pair<KeyIntervalAStar::ReachabilityResult, std::unordered_map<IntervalKey, IntervalKey, IntervalKeyHash>> processNeighborChains(const VerticalInterval& interval,
                                                                 const Vertex& vertex,
                                                                 const Vertex& target,
                                                                 bool isStart) const;
    
    // 新增：安全的vertical interval访问
    std::optional<VerticalInterval> getVerticalInterval(const IntervalKey& key) const;

    // 处理邻居链查找key interval（统一版本）
    // 返回值：true表示找到了target vertex，false表示继续寻找key interval
    bool processNeighborChain(const IntervalKey& currentKey,
                                   const Vertex& vertex,
                                   const Vertex& target,
                                   bool isStart,
                                   std::unordered_map<IntervalKey, IntervalKey, IntervalKeyHash>& keyIntervals,
                                   bool isLeftDirection) const;

    // 检查vertex是否在某个key interval中
    bool isVertexInKeyInterval(const Vertex& vertex, const IntervalKey& key) const;

    // 计算g值和h值
    std::pair<double, double> calcGAndH(const std::vector<Vertex>& mustPassPoints,
                                      const IntervalKey& evaluationInterval,
                                      const Vertex& end) const;

    // 构建最终路径
    std::vector<Vertex> constructPath(std::vector<Vertex> waypoints) const;

    // 处理终点key interval
    SearchNode handleTargetKeyInterval(const SearchNode& current,
                                const IntervalKey& targetFromDirectNeighbor,
                                const Vertex& target);

    // 处理邻居节点
    SearchNode handleNeighbor(const SearchNode& current,
                       const NeighborTriple& neighborTriple,
                       const Vertex& target);

    
    SearchNode directTransition(const SearchNode& current,
                        const NeighborTriple& neighborTriple,
                        const Vertex& transitionVertex,
                        const Vertex& target);

    // 处理key points
    SearchNode handleUpDownVertex(const SearchNode& current,
                        const NeighborTriple& neighborTriple,
                        const Vertex& target);

    // 插入节点到openList，包含g值比较逻辑
    void insertNode(const SearchNode& node,
                   std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>>& openList,
                   std::unordered_map<IntervalKey, double, IntervalKeyHash>& gScore);

    // 检查是否需要添加up/down vertex
    bool checkAndAddUpDownVertex(const Vertex& targetVertex, 
                                const Vertex& lastWaypoint,
                                std::vector<Vertex>& waypoints,
                                const SearchNode& current);

    // 寻找transition vertex
    std::optional<Vertex> findTransitionVertex(const IntervalKey& keyInterval,
                                              const IntervalKey& directNeighbor1,
                                              const IntervalKey& directNeighbor2) const;

}; 