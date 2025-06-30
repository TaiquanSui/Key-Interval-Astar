#pragma once

#include <vector>
#include <set>
#include <map>
#include "../basic/Vertex.h"
#include "../utilities/GridUtility.h"

class Preprocess2 {
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
            // 先比较x坐标
            if (point.x != other.point.x) {
                return point.x < other.point.x;
            }
            // 再比较y坐标
            if (point.y != other.point.y) {
                return point.y < other.point.y;
            }
            // 最后比较方向
            return direction < other.direction;
        }
    };

    // 定义区间结构
    struct Interval {
        int start;
        int end;
        Trend start_mark;  // 起点趋势标记
        Trend end_mark;    // 终点趋势标记
        std::set<KeyPoint> keyPoints;  // 该区间内的关键点

        Interval(int s, int e) : 
            start(s), end(e), 
            start_mark(Trend::UNCHANGED), 
            end_mark(Trend::UNCHANGED) {}
    };

    // 定义子空间结构
    struct Subspace {
        std::vector<Interval> verticalIntervals;  // 子空间包含的竖向区间
        std::set<KeyPoint> keyPoints;  // 子空间的关键点
        std::set<size_t> neighbors;    // 邻居子空间的索引
    };

    // 构造函数
    Preprocess2(const std::vector<std::vector<int>>& grid);

    // 执行预处理
    void preprocess();

private:
    // 地图数据
    const std::vector<std::vector<int>>& grid_;
    // 地图尺寸
    int width_;
    int height_;
    
    // 存储所有子空间
    std::vector<Subspace> subspaces_;

    // 存储区间到子空间的映射
    std::map<std::pair<int, int>, size_t> intervalToSubspace_;

    // 水平扫描
    void horizontalScan();

    // 获取一列的最大可移动空间
    void getMaxMovableSpace(int y, std::vector<Interval>& intervals) const;

    // 获取一行的最大可移动空间
    void getMaxMovableSpaceVertical(int x, std::vector<Interval>& intervals) const;

    // 更新趋势标记
    void updateTrend(int curr, int prev, Trend& mark, Trend& prevMark) const;

    // 检查趋势变化并记录关键点
    void checkTrendChange(const Interval& prev, const Interval& curr, 
                         int x, int y, std::set<KeyPoint>& keyPoints, Direction dir) const;

    // 构建子空间
    void buildSubspaces();

    // 构建子空间之间的邻居关系
    void buildNeighbors();

    // 检查两个子空间是否相邻
    bool areSubspacesNeighbors(const Subspace& s1, const Subspace& s2) const;

    // 收集关键点
    void collectKeyPoints(std::vector<std::vector<KeyPoint>>& verticalKeyPoints,
                         std::vector<std::vector<KeyPoint>>& horizontalKeyPoints);

    // 合并关键点到子空间
    void mergeKeyPointsToSubspaces(const std::vector<std::vector<KeyPoint>>& verticalKeyPoints,
                                  const std::vector<std::vector<KeyPoint>>& horizontalKeyPoints);

    // 检查区间是否属于某个子空间
    bool isIntervalInSubspace(const Interval& interval, size_t subspaceIndex) const;
}; 