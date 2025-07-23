#pragma once

#include <vector>
#include <set>
#include <map>
#include <unordered_map>
#include <iostream>
#include <optional>
#include "../basic/Vertex.h"
#include "../utilities/GridUtility.h"


class Preprocess {
public:
    // 定义趋势枚举
    enum class Trend {
        UNCHANGED = 0,
        DECREASING = 1,
        INCREASING = 2
    };

    const char* trendToString(Trend t) {
        switch (t) {
            case Trend::UNCHANGED:   return "UNCHANGED";
            case Trend::DECREASING:  return "DECREASING";
            case Trend::INCREASING:  return "INCREASING";
            default: return "UNKNOWN";
        }
    }

    // 定义关键点方向
    enum class Direction {
        UP = 0,     // 可从上方经过
        DOWN = 1,   // 可从下方经过
        LEFT = 2,   // 可从左侧经过
        RIGHT = 3   // 可从右侧经过
    };

    // 定义关键点结构
    struct KeyPoint {
        Vertex point;
        Direction direction;

        KeyPoint(const Vertex& p, Direction d) : point(p), direction(d) {}
        
        bool operator<(const KeyPoint& other) const {
            if (point.x != other.point.x) {
                return point.x < other.point.x;
            }
            if (point.y != other.point.y) {
                return point.y < other.point.y;
            }
            return direction < other.direction;
        }
    };

    // 定义用于扫描的区间结构
    struct Interval {
        int start;
        int end;
        Trend start_mark;  // 起点趋势标记
        Trend end_mark;    // 终点趋势标记

        Interval(int s, int e) : 
            start(s), end(e), 
            start_mark(Trend::UNCHANGED), 
            end_mark(Trend::UNCHANGED) {}
    };

    // 定义用于hashmap的key结构
    struct IntervalKey {
        int y;
        int start;
        int end;
        
        bool operator==(const IntervalKey& other) const {
            return y == other.y && start == other.start && end == other.end;
        }

        bool operator<(const IntervalKey& other) const {
            if (y != other.y) return y < other.y;
            if (start != other.start) return start < other.start;
            return end < other.end;
        }
    };

    // 定义hash函数
    struct IntervalKeyHash {
        size_t operator()(const IntervalKey& key) const {
            return std::hash<int>()(key.y) ^ 
                   (std::hash<int>()(key.start) << 1) ^ 
                   (std::hash<int>()(key.end) << 2);
        }
    };

    // 定义垂直区间结构
    struct VerticalInterval {
        int y;              // y坐标
        int start;          // x坐标起点
        int end;            // x坐标终点
        std::optional<Vertex> upVertex;             // 垂直方向up关键点
        std::optional<Vertex> downVertex;           // 垂直方向down关键点
        std::vector<KeyPoint> horizontalKeyPoints;  // 水平方向的关键点（LEFT/RIGHT）
        std::vector<IntervalKey> leftNeighbors;     // 左侧邻居
        std::vector<IntervalKey> rightNeighbors;    // 右侧邻居
        std::optional<std::pair<IntervalKey, IntervalKey>> leftKeyInterval;   // (key interval, key interval direct neighbor)
        std::optional<std::pair<IntervalKey, IntervalKey>> rightKeyInterval;  // (key interval, key interval direct neighbor)
        
        // 带参数的构造函数
        VerticalInterval(int y_pos, int s, int e) : 
            y(y_pos), start(s), end(e) {}

        VerticalInterval() : y(0), start(0), end(0) {}
    };

    // 定义邻居三元组结构
    struct NeighborTriple {
        IntervalKey currentDirectNeighbor;    // 当前interval的直接邻居
        IntervalKey neighborDirectNeighbor;       // 到达邻居key interval的直接邻居
        IntervalKey neighborKeyInterval;      // 邻居key interval
        
        NeighborTriple(const IntervalKey& current, const IntervalKey& path, const IntervalKey& neighbor)
            : currentDirectNeighbor(current), neighborDirectNeighbor(path), neighborKeyInterval(neighbor) {}
        
        bool operator==(const NeighborTriple& other) const {
            return currentDirectNeighbor == other.currentDirectNeighbor &&
                   neighborDirectNeighbor == other.neighborDirectNeighbor &&
                   neighborKeyInterval == other.neighborKeyInterval;
        }
        
        bool operator!=(const NeighborTriple& other) const {
            return !(*this == other);
        }
    };

    // 定义关键区间结构
    struct KeyInterval {
        const VerticalInterval* verticalInterval;  // 指向对应的VerticalInterval
        // 直接存储邻居三元组集合
        std::vector<NeighborTriple> neighbors;
        // 存储直接邻居之间的transition vertices
        std::unordered_map<IntervalKey, std::unordered_map<IntervalKey, Vertex, IntervalKeyHash>, IntervalKeyHash> transitionVertices;

        KeyInterval(const VerticalInterval* vInterval) : verticalInterval(vInterval) {}
        KeyInterval() = default;
        
        // 便捷访问方法
        int getY() const { return verticalInterval->y; }
        int getStart() const { return verticalInterval->start; }
        int getEnd() const { return verticalInterval->end; }
        const std::vector<KeyPoint>& getHorizontalKeyPoints() const { return verticalInterval->horizontalKeyPoints; }
        std::optional<Vertex> getUpVertex() const { return verticalInterval->upVertex; }
        std::optional<Vertex> getDownVertex() const { return verticalInterval->downVertex; }
    };

    // 构造函数
    Preprocess(const std::vector<std::vector<int>>& grid);

    // 执行预处理
    void preprocess();
    double getPreprocessTime() const { return preprocess_time_; }

    // 获取地图尺寸
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }

    // 获取key intervals
    const std::unordered_map<IntervalKey, KeyInterval, IntervalKeyHash>& getKeyIntervals() const { 
        return keyIntervals_; 
    }

    // 获取vertical intervals
    const std::unordered_map<int, std::unordered_map<int, VerticalInterval>>& getVerticalIntervals() const { 
        return verticalIntervals_; 
    }

    // 根据key查找KeyInterval
    const std::optional<KeyInterval> findKeyInterval(const IntervalKey& key) const {
        auto it = keyIntervals_.find(key);
        return it != keyIntervals_.end() ? std::optional<KeyInterval>(it->second) : std::nullopt;
    }

    const std::optional<KeyInterval> findKeyInterval(const VerticalInterval& interval) const {
        for(const auto& [_, keyInterval] : keyIntervals_) {
            if(keyInterval.verticalInterval == &interval) {
                return std::optional<KeyInterval>(keyInterval);
            }
        }
        return std::nullopt;
    }

    // 查找包含vertex的vertical interval
    std::optional<VerticalInterval> findContainingVerticalInterval(const Vertex& vertex) const;
    // 判断vertex是否在某个key interval中
    static bool isVertexInKeyInterval(const Vertex& vertex, const IntervalKey& key);
    // 寻找transition vertex
    std::optional<Vertex> findTransitionVertex(const IntervalKey& keyInterval,
                                              const IntervalKey& directNeighbor1,
                                              const IntervalKey& directNeighbor2) const;

    // 通用扫描方法
    void processScan(bool isVertical);

    // 处理关键点
    // void processKeyPoint(std::vector<Interval>& currentIntervals, 
    //                     std::vector<Interval>& nextIntervals, bool isVertical, int fixed);

    // 处理邻居关系
    void processNeighborPair(const IntervalKey& key,
                           const std::vector<IntervalKey>& directNeighbors,
                           Direction dir);

    // 处理transition vertices
    void processTransitionVertices(const IntervalKey& key,
                            const VerticalInterval& currentInterval,
                            const std::vector<IntervalKey>& directNeighbors,
                                    Direction dir);

    // 获取地图数据
    const std::vector<std::vector<int>>& getGrid() const { return grid_; }
    
    // 获取预处理占用的内存（字节）
    size_t getMemoryUsage() const;

private:
    // 地图数据
    const std::vector<std::vector<int>>& grid_;
    // 地图尺寸
    int width_;
    int height_;
    
    // 存储所有vertical interval
    std::unordered_map<int, std::unordered_map<int, VerticalInterval>> verticalIntervals_;
    
    // 存储所有key interval（使用hashmap）
    std::unordered_map<IntervalKey, KeyInterval, IntervalKeyHash> keyIntervals_;

    double preprocess_time_ = 0.0;

    // 获取最大可移动空间
    void getMaxMovableSpace(int fixed, int range, bool isVertical, std::vector<Interval>& intervals) const;

    // 构建key interval
    void buildKeyIntervals();

    // 构建key interval之间的邻居关系
    void buildNeighbors();

    // 更新区间趋势
    void processCurrentIntervals(std::vector<Interval>& prevIntervals, std::vector<Interval>& currentIntervals, 
                            std::vector<Interval>& nextIntervals, bool isVertical, int fixed);
    
}; 


// 在Preprocess类外部加上
inline std::ostream& operator<<(std::ostream& os, const Preprocess::IntervalKey& key) {
    os << "[" << key.y << "," << key.start << "," << key.end << "]";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const Preprocess::Direction& direction) {
    os << "[" << direction << "]";
    return os;
}   

inline std::ostream& operator<<(std::ostream& os, const Preprocess::KeyPoint& keyPoint) {
    os << "[" << keyPoint.point.x << "," << keyPoint.point.y << "," << keyPoint.direction << "]";
    return os;
}   

inline std::ostream& operator<<(std::ostream& os, const Preprocess::Interval& interval) {
    os << "[" << interval.start << "," << interval.end << "]";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const Preprocess::VerticalInterval& verticalInterval) {
    os << "[" << verticalInterval.y << "," << verticalInterval.start << "," << verticalInterval.end << "]";
    return os;
}
// 为std::pair<IntervalKey, IntervalKey>添加operator<<
inline std::ostream& operator<<(std::ostream& os, const std::pair<Preprocess::IntervalKey, Preprocess::IntervalKey>& pair) {
    os << "(" << pair.first << "->" << pair.second << ")";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const Preprocess::NeighborTriple& triple) {
    os << "(" << triple.currentDirectNeighbor << "->" << triple.neighborDirectNeighbor << "->" << triple.neighborKeyInterval << ")";
    return os;
}

