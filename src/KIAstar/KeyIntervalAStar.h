#pragma once

#include <vector>
#include <set>
#include <queue>
#include <unordered_map>
#include <optional>
#include "../basic/Vertex.h"
#include "../utilities/GridUtility.h"
#include "../solver/SolverInterface.h"
#include "Preprocess.h"

using IntervalKey = Preprocess::IntervalKey;
using KeyInterval = Preprocess::KeyInterval;
using IntervalKeyHash = Preprocess::IntervalKeyHash;
using VerticalInterval = Preprocess::VerticalInterval;
using NeighborTriple = Preprocess::NeighborTriple;

class KeyIntervalAStar : public SolverInterface {
public:
    // 定义搜索节点结构
    struct SearchNode {
        std::optional<IntervalKey> keyInterval;  // key interval的key
        double g;                 // 从起点到当前节点的代价
        double f;                 // 估计的总代价
        std::optional<IntervalKey> parent;  // 父节点的key
        std::optional<IntervalKey> fromDirectNeighbor;  // 当前直接邻居的key
        std::optional<Vertex> upVertex;          // 记录y值最小的direction为up的key point
        std::optional<Vertex> downVertex;        // 记录y值最大的direction为down的key point
        std::vector<Vertex> waypoints;  // 必须经过的点集合
        bool isTarget;              // 是否到达终点

        SearchNode(const std::optional<IntervalKey>& key, double g_val, double f_val, 
                  const std::optional<IntervalKey>& p,
                  const std::optional<IntervalKey>& fromDirectNeighbor,
                  const std::optional<Vertex>& up,
                  const std::optional<Vertex>& down,
                  const std::vector<Vertex>& waypoints,
                  bool is_end = false)
            : keyInterval(key), g(g_val), f(f_val), parent(p),
              fromDirectNeighbor(fromDirectNeighbor),
              upVertex(up), downVertex(down),
              waypoints(waypoints), isTarget(is_end) {}

        // 用于优先队列的比较
        bool operator>(const SearchNode& other) const {
            if (f == other.f) {
                return g < other.g;
            }
            return f > other.f;
        }
    };
    
    struct KeyIntervalQueryResult {
        bool directlyReachable;
        std::optional<IntervalKey> startK;    // 起点所在的关键区间
        std::optional<IntervalKey> targetK;   // 终点所在的关键区间
        std::optional<std::pair<IntervalKey, IntervalKey>> startLeft;
        std::optional<std::pair<IntervalKey, IntervalKey>> startRight;
        std::optional<std::pair<IntervalKey, IntervalKey>> targetLeft;
        std::optional<std::pair<IntervalKey, IntervalKey>> targetRight;
    };

    
    
    // 构造函数
    KeyIntervalAStar(const Preprocess& preprocess, const std::string& name = "KeyIntervalAStar");
    // 新增：默认构造函数，用于延迟初始化
    KeyIntervalAStar(const std::string& name = "KeyIntervalAStar");

    // 实现SolverInterface接口
    std::vector<Vertex> search(const Vertex& start, const Vertex& target) override;
    std::string get_name() const override { return name_; }
    int getExpandedNodes() const override { return expanded_nodes_; }
    void resetExpandedNodes() override { expanded_nodes_ = 0; }
    double getSearchTime() const override { return search_time_; }
    void resetSearchTime() override { search_time_ = 0.0; }
    
    // 实现内存监控接口
    size_t getPreprocessMemoryUsage() const override { return preprocess_memory_usage_; }
    size_t getSearchMemoryIncrease() const override { return search_memory_increase_; }
    void resetSearchMemoryUsage() override { search_memory_increase_ = 0; }
    // 实现预处理时间接口
    double getPreprocessTime() const override { return preprocess_time_; }
    
    // 实现预处理接口
    void preprocess(const std::vector<std::vector<int>>& grid) override;

private:
    const Preprocess* preprocess_;  // 改为指针，支持延迟初始化
    std::string name_;              // 算法名称
    int expanded_nodes_ = 0;        // 扩展节点计数
    double search_time_ = 0.0;      // 搜索时间
    size_t preprocess_memory_usage_ = 0;  // 预处理占用的内存（字节）
    size_t search_memory_increase_ = 0;   // 搜索过程中增加的内存（字节）
    double preprocess_time_ = 0.0;        // 预处理时间（秒）
    std::unique_ptr<Preprocess> preprocess_ptr_; // 拥有Preprocess对象

    // 计算g值和h值
    std::pair<double, double> calcGAndH(const std::vector<Vertex>& mustPassPoints,
                                      const IntervalKey& evaluationInterval,
                                      const Vertex& end) const;

    // 构建最终路径
    std::vector<Vertex> constructPath(std::vector<Vertex> waypoints) const;

        // 处理终点key intervals
    bool handleTargetKeyIntervals(const SearchNode& current, const Vertex& target,
                                const KeyIntervalQueryResult& queryResult,
                                std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>>& openList,
                                std::unordered_map<IntervalKey, double, IntervalKeyHash>& gScore);

    // 处理终点key interval
    SearchNode handleTargetKeyInterval(const SearchNode& current,
                                const IntervalKey& targetFromDirectNeighbor,
                                const Vertex& target);

    // 查询key intervals
    KeyIntervalQueryResult queryKeyIntervals(const Vertex& start, const Vertex& target) const;

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

    // 初始化起点节点
    void initializeStartNode(const Vertex& start, const Vertex& target,
                           const KeyIntervalQueryResult& queryResult,
                           std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>>& openList,
                           std::unordered_map<IntervalKey, double, IntervalKeyHash>& gScore);

    
}; 