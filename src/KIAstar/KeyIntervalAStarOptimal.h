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

class KeyIntervalAStarOptimal : public SolverInterface {
public:
    // define search node structure
    struct SearchNode {
        std::optional<IntervalKey> keyInterval;  // key of key interval
        double g;                 // cost from start to current node
        double f;                 // estimated total cost
        std::optional<IntervalKey> parent;  // key of parent node
        std::optional<IntervalKey> fromDirectNeighbor;  // key of current direct neighbor
        std::optional<Vertex> upVertex;          // key point with smallest y value and direction up
        std::optional<Vertex> downVertex;        // key point with largest y value and direction down
        std::vector<Vertex> waypoints;  // set of points that must be passed
        bool isTarget;              // whether reached target

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

        // for priority queue comparison
        bool operator>(const SearchNode& other) const {
            if (f == other.f) {
                return g < other.g;
            }
            return f > other.f;
        }
    };
    
    struct KeyIntervalQueryResult {
        bool directlyReachable;
        std::optional<IntervalKey> startK;    // key interval of start point
        std::optional<IntervalKey> targetK;   // key interval of target point
        std::optional<std::pair<IntervalKey, IntervalKey>> startLeft;
        std::optional<std::pair<IntervalKey, IntervalKey>> startRight;
        std::optional<std::pair<IntervalKey, IntervalKey>> targetLeft;
        std::optional<std::pair<IntervalKey, IntervalKey>> targetRight;
    };

    // new: StateKey structure for more detailed state distinction
    struct StateKey {
        IntervalKey intervalKey;
        Vertex lastWaypoint;
        std::optional<Vertex> upVertex;
        std::optional<Vertex> downVertex;
        bool operator==(const StateKey& other) const {
            return intervalKey == other.intervalKey &&
                   lastWaypoint == other.lastWaypoint &&
                   upVertex == other.upVertex &&
                   downVertex == other.downVertex;
        }
    };
    
    struct StateKeyHash {
        size_t operator()(const StateKey& key) const {
            size_t h1 = IntervalKeyHash{}(key.intervalKey);
            size_t h2 = std::hash<int>()(key.lastWaypoint.x) ^ (std::hash<int>()(key.lastWaypoint.y) << 1);
            size_t h3 = key.upVertex ? (std::hash<int>()(key.upVertex->x) ^ (std::hash<int>()(key.upVertex->y) << 1)) : 0;
            size_t h4 = key.downVertex ? (std::hash<int>()(key.downVertex->x) ^ (std::hash<int>()(key.downVertex->y) << 1)) : 0;
            return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3);
        }
    };
    
    // constructor
    KeyIntervalAStarOptimal(const Preprocess& preprocess, const std::string& name = "KeyIntervalAStarOptimal");
    // new: default constructor, for lazy initialization
    KeyIntervalAStarOptimal(const std::string& name = "KeyIntervalAStarOptimal");

    // implement SolverInterface interface
    std::vector<Vertex> search(const Vertex& start, const Vertex& target) override;
    std::string get_name() const override { return name_; }
    int getExpandedNodes() const override { return expanded_nodes_; }
    void resetExpandedNodes() override { expanded_nodes_ = 0; }
    double getSearchTime() const override { return search_time_; }
    void resetSearchTime() override { search_time_ = 0.0; }
    
    // implement memory monitoring interface
    size_t getPreprocessMemoryUsage() const override { return preprocess_memory_usage_; }
    size_t getSearchMemoryIncrease() const override { return search_memory_increase_; }
    void resetSearchMemoryUsage() override { search_memory_increase_ = 0; }
    // implement preprocessing time interface
    double getPreprocessTime() const override { return preprocess_time_; }
    
    // implement preprocessing interface
    void preprocess(const std::vector<std::vector<int>>& grid) override;

private:
    const Preprocess* preprocess_;  // change to pointer, support lazy initialization
    std::string name_;              // algorithm name
    int expanded_nodes_ = 0;        // expanded nodes count
    double search_time_ = 0.0;      // search time
    size_t preprocess_memory_usage_ = 0;  // memory usage of preprocessing (bytes)
    size_t search_memory_increase_ = 0;   // memory usage of search (bytes)
    double preprocess_time_ = 0.0;        // preprocessing time (seconds)
    std::unique_ptr<Preprocess> preprocess_ptr_; // own Preprocess object

    // calculate g value and h value
    std::pair<double, double> calcGAndH(const std::vector<Vertex>& mustPassPoints,
                                      const IntervalKey& evaluationInterval,
                                      const Vertex& end) const;

    // construct final path
    std::vector<Vertex> constructPath(std::vector<Vertex> waypoints) const;

    // handle target key intervals
    bool handleTargetKeyIntervals(const SearchNode& current, const Vertex& target,
                                const KeyIntervalQueryResult& queryResult,
                                std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>>& openList,
                                std::unordered_map<StateKey, double, StateKeyHash>& gScore);

    // handle target key interval
    SearchNode handleTargetKeyInterval(const SearchNode& current,
                                const IntervalKey& targetFromDirectNeighbor,
                                const Vertex& target);

    // query key intervals
    KeyIntervalQueryResult queryKeyIntervals(const Vertex& start, const Vertex& target) const;

    // handle neighbor node
    SearchNode handleNeighbor(const SearchNode& current,
                       const NeighborTriple& neighborTriple,
                       const Vertex& target);

    SearchNode directTransition(const SearchNode& current,
                        const NeighborTriple& neighborTriple,
                        const Vertex& transitionVertex,
                        const Vertex& target);

    // handle key points
    SearchNode handleUpDownVertex(const SearchNode& current,
                        const NeighborTriple& neighborTriple,
                        const Vertex& target);

    // insert node to openList, include g value comparison logic (using StateKey)
    void insertNode(const SearchNode& node,
                   std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>>& openList,
                   std::unordered_map<StateKey, double, StateKeyHash>& gScore);

    // check if need to add up/down vertex
    bool checkAndAddUpDownVertex(const Vertex& targetVertex, 
                                const Vertex& lastWaypoint,
                                std::vector<Vertex>& waypoints,
                                const SearchNode& current);

    // initialize start node
    void initializeStartNode(const Vertex& start, const Vertex& target,
                           const KeyIntervalQueryResult& queryResult,
                           std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>>& openList,
                           std::unordered_map<StateKey, double, StateKeyHash>& gScore);
}; 