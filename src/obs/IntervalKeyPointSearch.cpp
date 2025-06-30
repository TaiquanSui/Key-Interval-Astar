#include "IntervalKeyPointSearch.h"
#include "../utilities/GridUtility.h"
#include "../heuristic/Heuristic.h"
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <limits>

IntervalKeyPointSearch::IntervalKeyPointSearch(const std::vector<std::vector<int>>& grid)
    : grid_(grid), preprocess_(grid) {
    // 执行预处理
    preprocess_.preprocess();
}

std::vector<Vertex> IntervalKeyPointSearch::search(const Vertex& start, const Vertex& goal) {
    // 找到起始点和目标点所在的区间
    auto startInterval = findInterval(start);
    auto goalInterval = findInterval(goal);

    // 如果找不到区间，返回空路径
    if (startInterval.start == -1 || goalInterval.start == -1) {
        return {};
    }

    // 创建起始节点
    auto startNode = std::make_shared<SearchNode>(startInterval, startInterval, start.y);
    startNode->keyPoints.push_back(start);
    startNode->mustPoints.push_back(start);

    // 创建优先队列
    auto cmp = [](const std::shared_ptr<SearchNode>& a, const std::shared_ptr<SearchNode>& b) {
        return a->f > b->f;
    };
    std::priority_queue<std::shared_ptr<SearchNode>, 
                       std::vector<std::shared_ptr<SearchNode>>, 
                       decltype(cmp)> openList(cmp);

    // 创建已访问集合
    std::set<std::pair<int, int>> visited; // 存储(y, interval.start)对

    // 计算起始节点的f值
    startNode->f = calculateFValue(startNode, goal);

    openList.push(startNode);
    visited.insert({start.y, startInterval.start});

    while (!openList.empty()) {
        auto current = openList.top();
        openList.pop();

        // 如果到达目标区间
        if (current->currentInterval.start == goalInterval.start && 
            current->currentInterval.end == goalInterval.end) {
            return buildPath(current, start, goal);
        }

        // 获取联通区间
        auto connectedIntervals = getConnectedIntervals(current->currentInterval, current->y);
        for (const auto& interval : connectedIntervals) {
            // 检查是否已访问
            if (visited.count({current->y + 1, interval.start})) {
                continue;
            }

            // 创建新节点
            auto newNode = std::make_shared<SearchNode>(interval, current->currentInterval, 
                                                      current->y + 1, current);
            
            // 处理关键点
            for (const auto& keyPoint : interval.keyPoints) {
                // 检查关键点是否在父节点的同一侧
                bool isSameSide = true;
                for (const auto& parentKeyPoint : current->keyPoints) {
                    if (!isOnSameSide(keyPoint.point, parentKeyPoint, keyPoint.direction)) {
                        isSameSide = false;
                        break;
                    }
                }
                
                if (isSameSide) {
                    newNode->keyPoints.push_back(keyPoint.point);
                    newNode->mustPoints.push_back(keyPoint.point);
                }
            }

            // 计算f值
            newNode->f = calculateFValue(newNode, goal);

            openList.push(newNode);
            visited.insert({current->y + 1, interval.start});
        }
    }

    return {}; // 没有找到路径
}

Preprocess::Interval IntervalKeyPointSearch::findInterval(const Vertex& point) const {
    const auto& intervals = preprocess_.getIntervalsAtY(point.y);
    for (const auto& interval : intervals) {
        if (point.x >= interval.start && point.x <= interval.end) {
            return interval;
        }
    }
    return Preprocess::Interval(-1, -1); // 未找到区间
}

bool IntervalKeyPointSearch::isConnected(const Preprocess::Interval& a, const Preprocess::Interval& b) const {
    return !(a.end <= b.start || b.end <= a.start);
}

std::vector<Preprocess::Interval> IntervalKeyPointSearch::getConnectedIntervals(
    const Preprocess::Interval& interval, int y) const {
    std::vector<Preprocess::Interval> result;
    
    // 检查上方区间
    if (y > 0) {
        const auto& intervals = preprocess_.getIntervalsAtY(y - 1);
        for (const auto& nextInterval : intervals) {
            if (isConnected(interval, nextInterval)) {
                result.push_back(nextInterval);
            }
        }
    }
    
    // 检查下方区间
    if (y < grid_.size() - 1) {
        const auto& intervals = preprocess_.getIntervalsAtY(y + 1);
        for (const auto& nextInterval : intervals) {
            if (isConnected(interval, nextInterval)) {
                result.push_back(nextInterval);
            }
        }
    }
    
    return result;
}

bool IntervalKeyPointSearch::isOnSameSide(const Vertex& point, const Vertex& keyPoint,
                                        Preprocess::Direction dir) const {
    switch (dir) {
        case Preprocess::Direction::UP:
            return point.y > keyPoint.y;
        case Preprocess::Direction::DOWN:
            return point.y < keyPoint.y;
        case Preprocess::Direction::LEFT:
            return point.x < keyPoint.x;
        case Preprocess::Direction::RIGHT:
            return point.x > keyPoint.x;
        default:
            return false;
    }
}

std::vector<Vertex> IntervalKeyPointSearch::buildPath(
    const std::shared_ptr<SearchNode>& goalNode,
    const Vertex& start, const Vertex& goal) const {
    std::vector<Vertex> path;
    
    // 从目标节点回溯到起始节点
    auto current = goalNode;
    while (current) {
        // 添加必须点
        for (const auto& mustPoint : current->mustPoints) {
            path.push_back(mustPoint);
        }
        current = current->parent;
    }
    
    // 反转路径，使其从起点到终点
    std::reverse(path.begin(), path.end());
    
    // 在必须点之间使用A*寻找路径
    std::vector<Vertex> finalPath;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        auto subPath = findPathBetweenMustPoints(path[i], path[i + 1]);
        finalPath.insert(finalPath.end(), subPath.begin(), subPath.end());
    }
    
    return finalPath;
}

std::vector<Vertex> IntervalKeyPointSearch::findPathBetweenMustPoints(
    const Vertex& start, const Vertex& goal) const {
    // 使用A*算法在必须点之间寻找路径
    return a_star(start, goal, grid_);
}

double IntervalKeyPointSearch::calculateFValue(const std::shared_ptr<SearchNode>& node, 
                                             const Vertex& goal) const {
    // 为区间内的每个点计算f值，取最小值
    double minF = std::numeric_limits<double>::max();
    
    // 遍历区间内的每个点
    for (int x = node->currentInterval.start; x <= node->currentInterval.end; ++x) {
        Vertex point(x, node->y);
        
        // 计算从起点到当前点的g值
        double g = 0.0;
        auto current = node;
        while (current->parent) {
            // 计算区间之间的移动距离
            double intervalDistance = std::abs(current->y - current->parent->y);
            
            // 计算区间内的移动距离
            double currentCenter = (current->currentInterval.start + current->currentInterval.end) / 2.0;
            double parentCenter = (current->parent->currentInterval.start + current->parent->currentInterval.end) / 2.0;
            double horizontalDistance = std::abs(currentCenter - parentCenter);
            
            // 使用曼哈顿距离
            g += intervalDistance + horizontalDistance;
            
            current = current->parent;
        }
        
        // 计算当前点到目标点的h值
        double h = heuristic_manhattan(point, goal);
        
        // 更新最小f值
        minF = std::min(minF, g + h);
    }
    
    return minF;
} 