#include "Preprocess1.h"
#include "../action/Action.h"
#include <iostream>
#include <algorithm>

Preprocess1::Preprocess1(const std::vector<std::vector<int>>& grid)
    : grid_(grid), width_(grid[0].size()), height_(grid.size()) {
}

const std::vector<Preprocess1::ObstacleCluster>& Preprocess1::getClusters() const {
    return clusters_;
}

void Preprocess1::preprocess() {
    clusters_ = scanClusters();
}

std::vector<Preprocess1::ObstacleCluster> Preprocess1::scanClusters() const {
    std::vector<ObstacleCluster> clusters;
    std::vector<std::vector<bool>> visited(height_, std::vector<bool>(width_, false));

    // 遍历整个地图
    for (int y = 0; y < height_; ++y) {
        for (int x = 0; x < width_; ++x) {
            // 如果当前位置是障碍物且未被访问过
            if (!utils::isPassable(grid_, x, y) && !visited[y][x]) {
                std::set<Vertex> cluster;
                dfs(x, y, cluster, visited);
                if (!cluster.empty()) {
                    ObstacleCluster obstacleCluster;
                    obstacleCluster.cluster = cluster;
                    horizontalScan(cluster, obstacleCluster.keyPoints);
                    verticalScan(cluster, obstacleCluster.keyPoints);
                    clusters.push_back(obstacleCluster);
                }
            }
        }
    }

    return clusters;
}

void Preprocess1::dfs(int x, int y, std::set<Vertex>& cluster, std::vector<std::vector<bool>>& visited) const {
    // 检查是否已访问过或是可通行位置
    if (visited[y][x] || utils::isPassable(grid_, x, y)) {
        return;
    }

    // 标记为已访问
    visited[y][x] = true;
    cluster.insert(Vertex(x, y));

    // 使用Action中定义的8个方向
    for (const auto& dir : Action::DIRECTIONS_8) {
        int newX = x + dir.x;
        int newY = y + dir.y;
        dfs(newX, newY, cluster, visited);
    }
}


void Preprocess1::horizontalScan(const std::set<Vertex>& cluster, std::set<Vertex>& keyPoints) const {
    // 找到集群的y坐标范围
    int minY = height_, maxY = -1;
    for (const auto& v : cluster) {
        minY = std::min(minY, v.y);
        maxY = std::max(maxY, v.y);
    }

    // 从左到右扫描
    directionalHorizontalScan(cluster, keyPoints, std::max(0, minY - 1), maxY + 1);
    
    // 从右到左扫描
    directionalHorizontalScan(cluster, keyPoints, maxY + 1, std::max(0, minY - 1));
}

void Preprocess1::updateTrend(int curr, int prev, Trend& mark, Trend& prevMark) const {
    if (curr < prev) {
        mark = Trend::INCREASING;
    } else if (curr > prev) {
        mark = Trend::DECREASING;
    } else {
        mark = prevMark;
    }
}

void Preprocess1::checkTrendChange(const ClusterInterval& prev, const ClusterInterval& curr, 
                                int x, int y, std::set<Vertex>& keyPoints) const {
    if (prev.x1_mark == Trend::DECREASING && curr.x1_mark == Trend::INCREASING) {
        keyPoints.insert(Vertex(x, y));
    }
}

void Preprocess1::directionalHorizontalScan(const std::set<Vertex>& cluster, std::set<Vertex>& keyPoints,
                                    int startY, int endY) const {
    std::vector<ClusterInterval> prevIntervals;
    int step = (endY > startY) ? 1 : -1;
    int y = startY;

    while ((step > 0 && y <= endY && y < height_) ||
           (step < 0 && y >= endY && y >= 0)) {
        std::vector<ClusterInterval> intervals;
        getMaxMovableSpace(y, cluster, intervals);

        if (y != startY) {
            for (auto& currInterval : intervals) {
                std::vector<ClusterInterval> connectedIntervals;
                for (const auto& prevInterval : prevIntervals) {
                    if (!(currInterval.x2 < prevInterval.x1 || currInterval.x1 > prevInterval.x2)) {
                        connectedIntervals.push_back(prevInterval);
                    }
                }

                if (!connectedIntervals.empty()) {
                    // 找到联通区间中的最小x1和最大x2
                    ClusterInterval minX1Interval = connectedIntervals[0];
                    ClusterInterval maxX2Interval = connectedIntervals[0];
                    for (const auto& interval : connectedIntervals) {
                        if (interval.x1 < minX1Interval.x1) minX1Interval = interval;
                        if (interval.x2 > maxX2Interval.x2) maxX2Interval = interval;
                    }

                    updateTrend(currInterval.x1, minX1Interval.x1, currInterval.x1_mark, minX1Interval.x1_mark);
                    updateTrend(currInterval.x2, maxX2Interval.x2, currInterval.x2_mark, maxX2Interval.x2_mark);

                    checkTrendChange(minX1Interval, currInterval, currInterval.x1, y, keyPoints);
                    checkTrendChange(maxX2Interval, currInterval, currInterval.x2, y, keyPoints);
                }
            }
        }

        prevIntervals = intervals;
        y += step;
    }
}

void Preprocess1::verticalScan(const std::set<Vertex>& cluster, std::set<Vertex>& keyPoints) const {
    // 找到集群的x坐标范围
    int minX = width_, maxX = -1;
    for (const auto& v : cluster) {
        minX = std::min(minX, v.x);
        maxX = std::max(maxX, v.x);
    }

    // 从上到下扫描
    directionalVerticalScan(cluster, keyPoints, std::max(0, minX - 1), maxX + 1);
    
    // 从下到上扫描
    directionalVerticalScan(cluster, keyPoints, maxX + 1, std::max(0, minX - 1));
}

void Preprocess1::directionalVerticalScan(const std::set<Vertex>& cluster, std::set<Vertex>& keyPoints,
                                            int startX, int endX) const {
    std::vector<ClusterInterval> prevIntervals;
    int step = (endX > startX) ? 1 : -1;
    int x = startX;

    while ((step > 0 && x <= endX && x < width_) ||
           (step < 0 && x >= endX && x >= 0)) {
        std::vector<ClusterInterval> intervals;
        getMaxMovableSpaceVertical(x, cluster, intervals);

        if (x != startX) {
            for (auto& currInterval : intervals) {
                std::vector<ClusterInterval> connectedIntervals;
                for (const auto& prevInterval : prevIntervals) {
                    if (!(currInterval.x2 < prevInterval.x1 || currInterval.x1 > prevInterval.x2)) {
                        connectedIntervals.push_back(prevInterval);
                    }
                }

                if (!connectedIntervals.empty()) {
                    // 找到联通区间中的最小y1和最大y2
                    ClusterInterval minY1Interval = connectedIntervals[0];
                    ClusterInterval maxY2Interval = connectedIntervals[0];
                    for (const auto& interval : connectedIntervals) {
                        if (interval.x1 < minY1Interval.x1) minY1Interval = interval;
                        if (interval.x2 > maxY2Interval.x2) maxY2Interval = interval;
                    }

                    updateTrend(currInterval.x1, minY1Interval.x1, currInterval.x1_mark, minY1Interval.x1_mark);
                    updateTrend(currInterval.x2, maxY2Interval.x2, currInterval.x2_mark, maxY2Interval.x2_mark);

                    checkTrendChange(minY1Interval, currInterval, x, currInterval.x1, keyPoints);
                    checkTrendChange(maxY2Interval, currInterval, x, currInterval.x2, keyPoints);
                }
            }
        }

        prevIntervals = intervals;
        x += step;
    }
}

void Preprocess1::getMaxMovableSpace(int y, const std::set<Vertex>& cluster,
                                       std::vector<ClusterInterval>& intervals) const {
    intervals.clear();
    int start = 0;
    bool inInterval = false;

    for (int x = 0; x < width_; ++x) {
        bool isObstacle = cluster.count(Vertex(x, y)) > 0;
        
        if (!isObstacle) {
            if (!inInterval) {
                start = x;
                inInterval = true;
            }
        } else {
            if (inInterval) {
                intervals.emplace_back(start, x - 1);
                inInterval = false;
            }
        }
    }

    if (inInterval) {
        intervals.emplace_back(start, width_ - 1);
    }
}

void Preprocess1::getMaxMovableSpaceVertical(int x, const std::set<Vertex>& cluster,
                                               std::vector<ClusterInterval>& intervals) const {
    intervals.clear();
    int start = 0;
    bool inInterval = false;

    for (int y = 0; y < height_; ++y) {
        bool isObstacle = cluster.count(Vertex(x, y)) > 0;
        
        if (!isObstacle && utils::isPassable(grid_, x, y)) {
            if (!inInterval) {
                start = y;
                inInterval = true;
            }
        } else {
            if (inInterval) {
                intervals.emplace_back(start, y - 1);
                inInterval = false;
            }
        }
    }

    if (inInterval) {
        intervals.emplace_back(start, height_ - 1);
    }
}

