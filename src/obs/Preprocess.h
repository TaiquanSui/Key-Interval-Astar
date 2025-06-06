#pragma once

#include <vector>
#include <set>
#include <map>
#include <unordered_map>
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

    // 定义关键点方向
    enum class Direction {
        UP = 0,     // 从上方经过
        DOWN = 1,   // 从下方经过
        LEFT = 2,   // 从左侧经过
        RIGHT = 3   // 从右侧经过
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
        Trend prev_start_mark;  // 起点趋势标记
        Trend prev_end_mark;    // 终点趋势标记

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

    // 定义邻居对的比较函数
    struct NeighborPairLess {
        bool operator()(const std::pair<IntervalKey, IntervalKey>& a, 
                       const std::pair<IntervalKey, IntervalKey>& b) const {
            // 确保较小的IntervalKey总是在前面
            if (a.first < b.first) return true;
            if (b.first < a.first) return false;
            return a.second < b.second;
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
        std::set<KeyPoint> horizontalKeyPoints;  // 水平方向的关键点（LEFT/RIGHT）
        std::set<KeyPoint> verticalKeyPoints;    // 垂直方向的关键点（UP/DOWN）
        std::set<IntervalKey> leftNeighbors;     // 左侧邻居
        std::set<IntervalKey> rightNeighbors;    // 右侧邻居

        VerticalInterval(int y_pos, int s, int e) : 
            y(y_pos), start(s), end(e) {}
    };

    // 定义关键区间结构
    struct KeyInterval {
        int y;              // y坐标
        int start;          // x坐标起点
        int end;            // x坐标终点
        std::set<KeyPoint> horizontalKeyPoints;  // 水平方向的关键点（LEFT/RIGHT）
        std::set<KeyPoint> verticalKeyPoints;    // 垂直方向的关键点（UP/DOWN）
        std::set<IntervalKey> neighbors;         // 所有邻居
        // 存储邻居对之间的mustpasspoint
        std::unordered_map<IntervalKey, std::unordered_map<IntervalKey, Vertex, IntervalKeyHash>, IntervalKeyHash> neighborMustPassPoints;

        KeyInterval(const VerticalInterval& vInterval) : 
            y(vInterval.y), start(vInterval.start), end(vInterval.end),
            horizontalKeyPoints(vInterval.horizontalKeyPoints),
            verticalKeyPoints(vInterval.verticalKeyPoints) {}
    };

    // 构造函数
    Preprocess(const std::vector<std::vector<int>>& grid);

    // 执行预处理
    void preprocess();

    // 获取地图尺寸
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }

    // 获取key intervals
    const std::unordered_map<IntervalKey, KeyInterval, IntervalKeyHash>& getKeyIntervals() const { 
        return keyIntervals_; 
    }

    // 获取vertical intervals
    const std::vector<std::vector<VerticalInterval>>& getVerticalIntervals() const { 
        return verticalIntervals_; 
    }

    // 根据key查找KeyInterval
    const KeyInterval* findKeyInterval(const IntervalKey& key) const {
        auto it = keyIntervals_.find(key);
        return it != keyIntervals_.end() ? &(it->second) : nullptr;
    }

    // 处理垂直扫描
    void processVerticalScan(int y, std::vector<Interval>& prevColumnIntervals,std::vector<Interval>& currentColumnIntervals, std::vector<Interval>& nextColumnIntervals);

    // 处理水平扫描
    void processHorizontalScan(int x, std::vector<Interval>& prevRowIntervals,std::vector<Interval>& currentRowIntervals, std::vector<Interval>& nextRowIntervals);

    // 处理关键点
    void processKeyPoint(std::vector<Interval>& currentIntervals, 
                        std::vector<Interval>& nextIntervals, bool isVertical, int fixed);

    // 处理邻居关系
    void processNeighborPair(const IntervalKey& key,
                           const std::set<IntervalKey>& directNeighbors,
                           int step,
                           Direction dir);

    // 处理mustpasspoint
    void processMustPassPoint(const IntervalKey& key,
                            const IntervalKey& neighbor1,
                            const IntervalKey& neighbor2,
                            const KeyPoint& keyPoint);

    // 更新邻居关系
    void updateNeighborRelation(VerticalInterval& currentInterval,
                              const VerticalInterval& nextInterval,
                              int y,
                              int nextY,
                              size_t nextIndex);

private:
    // 地图数据
    const std::vector<std::vector<int>>& grid_;
    // 地图尺寸
    int width_;
    int height_;
    
    // 存储所有vertical interval
    std::vector<std::vector<VerticalInterval>> verticalIntervals_;

    
    // 存储所有key interval（使用hashmap）
    std::unordered_map<IntervalKey, KeyInterval, IntervalKeyHash> keyIntervals_;

    // 获取最大可移动空间
    void getMaxMovableSpace(int fixed, int range, bool isVertical, std::vector<Interval>& intervals) const;

    // 收集关键点
    void collectKeyPoints();

    // 构建key interval
    void buildKeyIntervals();

    // 构建key interval之间的邻居关系
    void buildNeighbors();

    // 更新区间趋势
    void updateIntervalTrends(std::vector<Interval>& prevIntervals, 
                            std::vector<Interval>& currentIntervals, bool isVertical, int fixed);
    
}; 