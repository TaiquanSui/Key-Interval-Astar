#pragma once

#include "../basic/Vertex.h"
#include "../astar/AStar.h"
#include "Preprocess1.h"
#include <vector>
#include <memory>
#include <set>

class IntervalKeyPointSearch {
public:
    // 搜索节点结构
    struct SearchNode {
        Preprocess1::Interval currentInterval;  // 当前区间
        Preprocess1::Interval parentInterval;   // 父节点区间
        int y;                                // y坐标
        std::shared_ptr<SearchNode> parent;   // 父节点
        std::vector<Vertex> keyPoints;        // 关键点
        std::vector<Vertex> mustPoints;       // 必须点
        double f;                             // f值 = g + h

        SearchNode(const Preprocess1::Interval& curr, const Preprocess1::Interval& parent, 
                  int y, std::shared_ptr<SearchNode> p = nullptr)
            : currentInterval(curr), parentInterval(parent), y(y), parent(p), f(0.0) {}
    };

    // 构造函数
    IntervalKeyPointSearch(const std::vector<std::vector<int>>& grid);

    // 搜索函数
    std::vector<Vertex> search(const Vertex& start, const Vertex& goal);

private:
    // 地图数据
    const std::vector<std::vector<int>>& grid_;
    // 预处理对象
    Preprocess1 preprocess_;
    
    // 找到点所在的区间
    Preprocess1::Interval findInterval(const Vertex& point) const;

    // 检查两个区间是否联通
    bool isConnected(const Preprocess1::Interval& a, const Preprocess1::Interval& b) const;

    // 获取联通区间
    std::vector<Preprocess1::Interval> getConnectedIntervals(
        const Preprocess1::Interval& interval, int y) const;

    // 检查点是否在关键点的同一侧
    bool isOnSameSide(const Vertex& point, const Vertex& keyPoint,
                     Preprocess1::Direction dir) const;

    // 计算f值
    double calculateFValue(const std::shared_ptr<SearchNode>& node, 
                          const Vertex& goal) const;
    // 构建路径
    std::vector<Vertex> buildPath(const std::shared_ptr<SearchNode>& goalNode,
                                 const Vertex& start, const Vertex& goal) const;

    // 在必须点之间寻找路径
    std::vector<Vertex> findPathBetweenMustPoints(
        const Vertex& start, const Vertex& goal) const;
}; 