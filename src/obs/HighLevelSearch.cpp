#include "HighLevelSearch.h"
#include <cmath>
#include <limits>

HighLevelSearch::HighLevelSearch(const std::vector<Preprocess2::Subspace>& subspaces)
    : subspaces_(subspaces) {}

std::vector<Vertex> HighLevelSearch::search(const Vertex& start, const Vertex& goal) {
    // 找到起点和终点所在的子空间
    size_t startSubspace = findSubspaceContainingPoint(start);
    size_t goalSubspace = findSubspaceContainingPoint(goal);

    if (startSubspace == std::numeric_limits<size_t>::max() || 
        goalSubspace == std::numeric_limits<size_t>::max()) {
        return {};  // 起点或终点不在任何子空间中
    }

    // 初始化开放列表和关闭列表
    std::vector<SubgraphNode> nodes;
    std::priority_queue<std::pair<double, size_t>, 
                       std::vector<std::pair<double, size_t>>,
                       std::greater<std::pair<double, size_t>>> openList;

    // 创建起点节点
    Vertex startEntrance, startExit;
    getEntranceAndExit(subspaces_[startSubspace], start, goal, startEntrance, startExit);
    nodes.emplace_back(startSubspace, startEntrance, startExit, 
                      0.0, calculateHeuristic(startExit, goal));
    openList.push({nodes.back().f, nodes.size() - 1});

    // 创建终点节点
    Vertex goalEntrance, goalExit;
    getEntranceAndExit(subspaces_[goalSubspace], start, goal, goalEntrance, goalExit);

    while (!openList.empty()) {
        // 获取f值最小的节点
        auto [f, nodeIndex] = openList.top();
        openList.pop();
        SubgraphNode& current = nodes[nodeIndex];

        if (current.closed) continue;
        current.closed = true;

        // 如果到达目标子空间
        if (current.subspaceIndex == goalSubspace) {
            // 构建路径
            std::vector<Vertex> path;
            path.push_back(goal);
            path.push_back(goalExit);
            
            size_t currentIndex = nodeIndex;
            while (currentIndex != 0) {
                path.push_back(nodes[currentIndex].entrance);
                currentIndex = nodes[currentIndex].parent;
            }
            
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        // 遍历当前子空间的所有邻居
        for (size_t neighborIndex : subspaces_[current.subspaceIndex].neighbors) {
            // 创建邻居节点
            Vertex neighborEntrance, neighborExit;
            getEntranceAndExit(subspaces_[neighborIndex], current.exit, goal, 
                             neighborEntrance, neighborExit);

            // 计算从当前节点到邻居节点的代价
            double newG = current.g + calculateCost(current, 
                SubgraphNode(neighborIndex, neighborEntrance, neighborExit, 0, 0));
            double newH = calculateHeuristic(neighborExit, goal);
            double newF = newG + newH;

            // 检查是否已经访问过这个邻居
            bool found = false;
            for (size_t i = 0; i < nodes.size(); ++i) {
                if (nodes[i].subspaceIndex == neighborIndex && !nodes[i].closed) {
                    found = true;
                    if (newG < nodes[i].g) {
                        nodes[i].g = newG;
                        nodes[i].f = newF;
                        nodes[i].parent = nodeIndex;
                        openList.push({newF, i});
                    }
                    break;
                }
            }

            // 如果邻居节点未被访问过，创建新节点
            if (!found) {
                nodes.emplace_back(neighborIndex, neighborEntrance, neighborExit, 
                                 newG, newH);
                nodes.back().parent = nodeIndex;
                openList.push({newF, nodes.size() - 1});
            }
        }
    }

    return {};  // 没有找到路径
}

size_t HighLevelSearch::findSubspaceContainingPoint(const Vertex& point) const {
    for (size_t i = 0; i < subspaces_.size(); ++i) {
        for (const auto& interval : subspaces_[i].verticalIntervals) {
            if (point.x >= interval.start && point.x <= interval.end &&
                point.y >= interval.start && point.y <= interval.end) {
                return i;
            }
        }
    }
    return std::numeric_limits<size_t>::max();
}

double HighLevelSearch::calculateHeuristic(const Vertex& a, const Vertex& b) const {
    // 使用欧几里得距离作为启发式函数
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

double HighLevelSearch::calculateCost(const SubgraphNode& from, const SubgraphNode& to) const {
    // 计算两个子空间之间的代价（这里使用欧几里得距离）
    return calculateHeuristic(from.exit, to.entrance);
}

void HighLevelSearch::getEntranceAndExit(const Preprocess2::Subspace& subspace, 
                                        const Vertex& from, const Vertex& to,
                                        Vertex& entrance, Vertex& exit) const {
    // 从关键点集合中选择最佳入口和出口
    selectBestPoints(subspace.keyPoints, from, to, entrance, exit);
}

void HighLevelSearch::selectBestPoints(const std::set<Preprocess2::KeyPoint>& keyPoints,
                                      const Vertex& from, const Vertex& to,
                                      Vertex& entrance, Vertex& exit) const {
    double minEntranceDist = std::numeric_limits<double>::max();
    double minExitDist = std::numeric_limits<double>::max();

    for (const auto& keyPoint : keyPoints) {
        // 选择距离起点最近的点作为入口
        double entranceDist = calculateHeuristic(keyPoint.point, from);
        if (entranceDist < minEntranceDist) {
            minEntranceDist = entranceDist;
            entrance = keyPoint.point;
        }

        // 选择距离终点最近的点作为出口
        double exitDist = calculateHeuristic(keyPoint.point, to);
        if (exitDist < minExitDist) {
            minExitDist = exitDist;
            exit = keyPoint.point;
        }
    }
} 