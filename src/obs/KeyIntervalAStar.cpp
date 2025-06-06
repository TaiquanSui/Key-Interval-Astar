#include "KeyIntervalAStar.h"
#include <algorithm>

// 使用Preprocess中定义的VerticalInterval
using VerticalInterval = Preprocess::VerticalInterval;
using IntervalKey = Preprocess::IntervalKey;

KeyIntervalAStar::KeyIntervalAStar(const Preprocess& preprocess)
    : preprocess_(preprocess) {
}

std::vector<Vertex> KeyIntervalAStar::search(const Vertex& start, const Vertex& target) {
    // 查找起点和终点附近的key intervals
    std::set<IntervalKey> startKeyIntervals = findNearestKeyIntervals(start, true);
    std::set<IntervalKey> endKeyIntervals = findNearestKeyIntervals(target, false);

    // 检查是否可以直接从起点到达终点
    if (startKeyIntervals.empty()) {
        // 如果找不到起点的key intervals，说明可以直接到达终点
        // TODO: 需要考虑起点和终点在同一个key interval的情况
        // TODO: 调用路径规划函数
        return {start, target};
    }

    // 初始化A*搜索
    std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> openList;
    std::unordered_map<IntervalKey, double> gScore;    // 记录g值

    // 将起点的key intervals加入openList
    for (const auto& startKey : startKeyIntervals) {
        std::vector<Vertex> initialMustPassPoints = {start};
        auto [g, h] = calcGAndH(initialMustPassPoints, startKey, target);
        SearchNode initialNode = SearchNode(startKey, g, g + h, 
                                                IntervalKey{-1, -1, -1},
                                                Vertex(-1, -1), Vertex(-1, -1),
                                                initialMustPassPoints);
        openList.push(initialNode);
        gScore[startKey] = g;
    }

    while (!openList.empty()) {
        SearchNode current = openList.top();
        openList.pop();

        // 检查是否到达终点
        if (current.isTarget) {
            return buildPath(current);
        }

        // 获取当前key interval
        const auto& currentInterval = preprocess_.getKeyIntervals().at(current.keyInterval);

        // 检查是否在终点key intervals中
        if (endKeyIntervals.find(current.keyInterval) != endKeyIntervals.end()) {
            handleTargetKeyInterval(current, target, openList);
            continue;
        }
        
        // 遍历当前key interval的邻居
        for (const auto& neighborKey : currentInterval.neighbors) {
            // 跳过父节点
            if (neighborKey == current.parent) continue;

            handleNeighbor(current, neighborKey, target, openList, gScore);
        }
    }

    // 如果没有找到路径，返回空路径
    return {};
}

void KeyIntervalAStar::handleNeighbor(const SearchNode& current,
                                    const IntervalKey& neighborKey,
                                    const Vertex& target,
                                    std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>>& openList,
                                    std::unordered_map<IntervalKey, double>& gScore) {
    const auto& currentInterval = preprocess_.getKeyIntervals().at(current.keyInterval);
    
    // 检查是否有mustpasspoint
    auto it = currentInterval.neighborMustPassPoints.find(current.parent);
    if (it != currentInterval.neighborMustPassPoints.end()) {
        auto it2 = it->second.find(neighborKey);
        if (it2 != it->second.end()) {
            handleMustPassPoint(current, neighborKey, it2->second, target, openList, gScore);
            return;
        }
    }
    
    // 处理key points
    const auto& neighborInterval = preprocess_.getKeyIntervals().at(neighborKey);
    handleKeyPoints(current, neighborKey, neighborInterval, target, openList, gScore);
}

void KeyIntervalAStar::handleMustPassPoint(const SearchNode& current,
                                         const IntervalKey& neighborKey,
                                         const Vertex& mustPassPoint,
                                         const Vertex& target,
                                         std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>>& openList,
                                         std::unordered_map<IntervalKey, double>& gScore) {
    std::vector<Vertex> newMustPassPoints = current.mustPassPoints;
    newMustPassPoints.push_back(mustPassPoint);
    auto [tentativeG, h] = calcGAndH(newMustPassPoints, neighborKey, target);
    
    if (gScore.find(neighborKey) == gScore.end() || tentativeG < gScore[neighborKey]) {
        gScore[neighborKey] = tentativeG;
        SearchNode newNode = SearchNode(neighborKey, tentativeG, tentativeG + h,
                                            current.keyInterval,
                                            Vertex(-1, -1), Vertex(-1, -1),
                                            newMustPassPoints);
        openList.push(newNode);
    }
}

void KeyIntervalAStar::handleKeyPoints(const SearchNode& current,
                                     const IntervalKey& neighborKey,
                                     const Preprocess::KeyInterval& neighborInterval,
                                     const Vertex& target,
                                     std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>>& openList,
                                     std::unordered_map<IntervalKey, double>& gScore) {
    Vertex tempUpVertex = current.upVertex;
    Vertex tempDownVertex = current.downVertex;
    bool shouldBreak = false;
    std::vector<Vertex> newMustPassPoints = current.mustPassPoints;
    
    // 检查邻居interval中的key points
    for (const auto& keyPoint : neighborInterval.verticalKeyPoints) {
        if (keyPoint.direction == Preprocess::Direction::UP) {
            if (current.downVertex.y != -1 && 
                keyPoint.point.y < current.downVertex.y && 
                current.mustPassPoints.back().y < current.downVertex.y) {
                newMustPassPoints.push_back(current.downVertex);
                newMustPassPoints.push_back(keyPoint.point);
                shouldBreak = true;
                break;
            }
            if (tempUpVertex.y == -1 || keyPoint.point.y < tempUpVertex.y) {
                tempUpVertex = keyPoint.point;
            }
        } else if (keyPoint.direction == Preprocess::Direction::DOWN) {
            if (current.upVertex.y != -1 && 
                keyPoint.point.y > current.upVertex.y && 
                current.mustPassPoints.back().y > current.upVertex.y) {
                newMustPassPoints.push_back(current.upVertex);
                newMustPassPoints.push_back(keyPoint.point);
                shouldBreak = true;
                break;
            }
            if (tempDownVertex.y == -1 || keyPoint.point.y > tempDownVertex.y) {
                tempDownVertex = keyPoint.point;
            }
        }
    }

    Vertex newUpVertex = shouldBreak ? newMustPassPoints.back() : 
                        (tempUpVertex.y != -1 ? tempUpVertex : Vertex(-1, -1));
    Vertex newDownVertex = shouldBreak ? newMustPassPoints.back() : 
                          (tempDownVertex.y != -1 ? tempDownVertex : Vertex(-1, -1));

    auto [tentativeG, h] = calcGAndH(newMustPassPoints, neighborKey, target);

    if (gScore.find(neighborKey) == gScore.end() || tentativeG < gScore[neighborKey]) {
        gScore[neighborKey] = tentativeG;
        SearchNode newNode = SearchNode(neighborKey, tentativeG, tentativeG + h,
                                            current.keyInterval,
                                            newUpVertex, newDownVertex,
                                            newMustPassPoints);
        openList.push(newNode);
    }
}


void KeyIntervalAStar::handleTargetKeyInterval(const SearchNode& current,
                                             const Vertex& target,
                                             std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>>& openList) {
    std::vector<Vertex> finalMustPassPoints = current.mustPassPoints;
    
    // 检查最后一个mustPassPoint到target vertex之间是否还有关键点
    Vertex currentStart = current.mustPassPoints.back();

    // 如果currentStart和target都在down vertex上方
    if (current.downVertex.y != -1 && 
        currentStart.y < current.downVertex.y && 
        target.y < current.downVertex.y) {
        finalMustPassPoints.push_back(current.downVertex);
        finalMustPassPoints.push_back(target);
    }
    
    // 如果currentStart和target都在up vertex下方
    if (current.upVertex.y != -1 && 
        currentStart.y > current.upVertex.y && 
        target.y > current.upVertex.y) {
        finalMustPassPoints.push_back(current.upVertex);
        finalMustPassPoints.push_back(target);
    }

    double g = current.g + calculateHeuristicDistance(finalMustPassPoints.back(), target);
    SearchNode targetNode(current.keyInterval, g, g, current.parent,
                         current.upVertex, current.downVertex, finalMustPassPoints, true);
    openList.push(targetNode);
}

std::set<IntervalKey> KeyIntervalAStar::findNearestKeyIntervals(const Vertex& vertex, bool isStart) {
    std::set<IntervalKey> keyIntervals;
    const auto& verticalIntervals = preprocess_.getVerticalIntervals();
    
    // 找到包含vertex的vertical interval
    for (const auto& interval : verticalIntervals[vertex.y]) {
        if (vertex.x >= interval.start && vertex.x <= interval.end) {
            // 处理左侧邻居
            for (const auto& leftNeighbor : interval.leftNeighbors) {
                for (int y = leftNeighbor.y; y >= 0; y -= 1) {
                    IntervalKey searchKey{y, leftNeighbor.start, leftNeighbor.end};
                    const auto* keyInterval = preprocess_.findKeyInterval(searchKey);
                    if (keyInterval) {
                        // 如果是起点，检查是否包含终点
                        if (isStart && isVertexInKeyInterval(vertex, searchKey)) {
                            keyIntervals.clear();
                            return keyIntervals;
                        }
                        keyIntervals.insert(searchKey);
                    }
                }
            }
            
            // 处理右侧邻居
            for (const auto& rightNeighbor : interval.rightNeighbors) {
                for (int y = rightNeighbor.y; y < preprocess_.getHeight(); y += 1) {
                    IntervalKey searchKey{y, rightNeighbor.start, rightNeighbor.end};
                    const auto* keyInterval = preprocess_.findKeyInterval(searchKey);
                    if (keyInterval) {
                        // 如果是起点，检查是否包含终点
                        if (isStart && isVertexInKeyInterval(vertex, searchKey)) {
                            keyIntervals.clear();
                            return keyIntervals;
                        }
                        keyIntervals.insert(searchKey);
                    }
                }
            }
            break;  // 找到包含vertex的interval后就可以退出循环
        }
    }
    
    return keyIntervals;
}

bool KeyIntervalAStar::isVertexInKeyInterval(const Vertex& vertex, const IntervalKey& key) const {
    return vertex.y == key.y && 
           vertex.x >= key.start && 
           vertex.x <= key.end;
}

std::pair<double, double> KeyIntervalAStar::calcGAndH(const std::vector<Vertex>& mustPassPoints,
                                                    const IntervalKey& keyInterval,
                                                    const Vertex& target) const {
    double gValue = 0.0;
    double hValue = 0.0;
    
    // 计算mustPassPoints中相邻点之间的代价
    for (size_t i = 1; i < mustPassPoints.size(); ++i) {
        gValue += calculateHeuristicDistance(mustPassPoints[i-1], mustPassPoints[i]);
    }
    
    // 计算最后一段的代价
    if (!isVertexInKeyInterval(mustPassPoints.back(), keyInterval)) {
        // 否则根据目标vertex的位置计算代价
        if (target.x <= keyInterval.start) {
            // 如果目标x小于等于interval的start
            gValue += calculateHeuristicDistance(mustPassPoints.back(), Vertex(keyInterval.start, keyInterval.y));
            hValue = calculateHeuristicDistance(Vertex(keyInterval.start, keyInterval.y), target);
        } else if (target.x >= keyInterval.end) {
            // 如果目标x大于等于interval的end
            gValue += calculateHeuristicDistance(mustPassPoints.back(), Vertex(keyInterval.end, keyInterval.y));
            hValue = calculateHeuristicDistance(Vertex(keyInterval.end, keyInterval.y), target);
        } else {
            // 如果目标x在interval的范围内
            gValue += calculateHeuristicDistance(mustPassPoints.back(), Vertex(target.x, keyInterval.y));
            hValue = calculateHeuristicDistance(Vertex(target.x, keyInterval.y), target);
        }
    }
    
    return std::make_pair(gValue, hValue);
}

std::vector<Vertex> KeyIntervalAStar::buildPath(const SearchNode& targetNode) const {
    // // 获取最后一个interval
    // const auto& lastInterval = preprocess_.getKeyIntervals().at(endNode.keyInterval);
    // std::vector<Vertex> finalMustPassPoints;
    
    // // 检查最后一个mustPassPoint到end vertex之间是否还有关键点
    // Vertex currentStart = endNode.mustPassPoints.back();

    // // 如果currentStart和end都在down vertex上方
    // if (endNode.downVertex.y != -1 && 
    //     currentStart.y < endNode.downVertex.y && 
    //     target.y < endNode.downVertex.y) {
    //     finalMustPassPoints.push_back(endNode.downVertex);
    //     finalMustPassPoints.push_back(target);
    // }
    
    // // 如果currentStart和end都在up vertex下方
    // if (endNode.upVertex.y != -1 && 
    //     currentStart.y > endNode.upVertex.y && 
    //     target.y > endNode.upVertex.y) {
    //     finalMustPassPoints.push_back(endNode.upVertex);
    //     finalMustPassPoints.push_back(target);
    // }
    
    // 使用mustPassPoints重建完整路径
    std::vector<Vertex> path;
    for (size_t i = 0; i < targetNode.mustPassPoints.size() - 1; ++i) {
        // TODO: 调用A*算法获取两个mustPassPoint之间的路径
        // 这里需要实现一个简单的A*算法来获取两点之间的路径
        // 暂时使用直线连接
        path.push_back(targetNode.mustPassPoints[i]);
    }

    return path;
} 