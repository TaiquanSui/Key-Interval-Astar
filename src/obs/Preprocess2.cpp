#include "Preprocess2.h"
#include <iostream>
#include <algorithm>
#include <queue>

Preprocess2::Preprocess2(const std::vector<std::vector<int>>& grid)
    : grid_(grid), width_(grid[0].size()), height_(grid.size()) {
}

void Preprocess2::preprocess() {
    // 收集所有关键点
    std::vector<std::vector<KeyPoint>> verticalKeyPoints(height_);
    std::vector<std::vector<KeyPoint>> horizontalKeyPoints(height_);
    collectKeyPoints(verticalKeyPoints, horizontalKeyPoints);

    // 构建子空间
    buildSubspaces();

    // 合并关键点到子空间
    mergeKeyPointsToSubspaces(verticalKeyPoints, horizontalKeyPoints);

    // 构建邻居关系
    buildNeighbors();
}

void Preprocess2::collectKeyPoints(std::vector<std::vector<KeyPoint>>& verticalKeyPoints,
                                std::vector<std::vector<KeyPoint>>& horizontalKeyPoints) {
    // 垂直扫描收集关键点
    for (int x = 0; x < width_; ++x) {
        std::vector<Interval> intervals;
        getMaxMovableSpaceVertical(x, intervals);
        
        if (!intervals.empty()) {
            intervals[0].start_mark = Trend::UNCHANGED;
            intervals[0].end_mark = Trend::UNCHANGED;
            
            for (size_t i = 1; i < intervals.size(); ++i) {
                updateTrend(intervals[i].start, intervals[i-1].start, 
                          intervals[i].start_mark, intervals[i-1].start_mark);
                updateTrend(intervals[i].end, intervals[i-1].end, 
                          intervals[i].end_mark, intervals[i-1].end_mark);
                
                if (intervals[i-1].start_mark == Trend::DECREASING && 
                    intervals[i].start_mark == Trend::INCREASING) {
                    verticalKeyPoints[intervals[i].start].emplace_back(
                        Vertex(x, intervals[i].start), Direction::UP);
                }
                if (intervals[i-1].end_mark == Trend::DECREASING && 
                    intervals[i].end_mark == Trend::INCREASING) {
                    verticalKeyPoints[intervals[i].end].emplace_back(
                        Vertex(x, intervals[i].end), Direction::DOWN);
                }
            }
        }
    }

    // 水平扫描收集关键点
    for (int y = 0; y < height_; ++y) {
        std::vector<Interval> intervals;
        getMaxMovableSpace(y, intervals);
        
        if (!intervals.empty()) {
            intervals[0].start_mark = Trend::UNCHANGED;
            intervals[0].end_mark = Trend::UNCHANGED;
            
            for (size_t i = 1; i < intervals.size(); ++i) {
                updateTrend(intervals[i].start, intervals[i-1].start, 
                          intervals[i].start_mark, intervals[i-1].start_mark);
                updateTrend(intervals[i].end, intervals[i-1].end, 
                          intervals[i].end_mark, intervals[i-1].end_mark);
                
                if (intervals[i-1].start_mark == Trend::DECREASING && 
                    intervals[i].start_mark == Trend::INCREASING) {
                    horizontalKeyPoints[y].emplace_back(
                        Vertex(intervals[i].start, y), Direction::LEFT);
                }
                if (intervals[i-1].end_mark == Trend::DECREASING && 
                    intervals[i].end_mark == Trend::INCREASING) {
                    horizontalKeyPoints[y].emplace_back(
                        Vertex(intervals[i].end, y), Direction::RIGHT);
                }
            }
        }
    }
}

void Preprocess2::buildSubspaces() {
    std::vector<std::vector<Interval>> verticalIntervals(height_);
    std::vector<std::vector<bool>> visited(height_);

    // 初始化垂直区间
    for (int x = 0; x < width_; ++x) {
        std::vector<Interval> intervals;
        getMaxMovableSpaceVertical(x, intervals);
        for (const auto& interval : intervals) {
            verticalIntervals[interval.start].push_back(interval);
        }
    }

    // 初始化访问标记
    for (int y = 0; y < height_; ++y) {
        visited[y].resize(verticalIntervals[y].size(), false);
    }

    // 构建子空间
    for (int y = 0; y < height_; ++y) {
        for (size_t i = 0; i < verticalIntervals[y].size(); ++i) {
            if (!visited[y][i]) {
                Subspace subspace;
                std::queue<std::pair<int, size_t>> q;
                q.push({y, i});
                visited[y][i] = true;

                while (!q.empty()) {
                    auto [current_y, current_idx] = q.front();
                    q.pop();
                    
                    const Interval& current = verticalIntervals[current_y][current_idx];
                    subspace.verticalIntervals.push_back(current);
                    intervalToSubspace_[{current_y, current_idx}] = subspaces_.size();

                    // 检查连通性
                    for (int x = current.start; x <= current.end; ++x) {
                        for (int dy = -1; dy <= 1; dy += 2) {
                            int next_y = current_y + dy;
                            if (next_y >= 0 && next_y < height_) {
                                for (size_t j = 0; j < verticalIntervals[next_y].size(); ++j) {
                                    if (!visited[next_y][j] && 
                                        x >= verticalIntervals[next_y][j].start && 
                                        x <= verticalIntervals[next_y][j].end) {
                                        q.push({next_y, j});
                                        visited[next_y][j] = true;
                                    }
                                }
                            }
                        }
                    }
                }
                
                subspaces_.push_back(subspace);
            }
        }
    }
}

void Preprocess2::mergeKeyPointsToSubspaces(const std::vector<std::vector<KeyPoint>>& verticalKeyPoints,
                                         const std::vector<std::vector<KeyPoint>>& horizontalKeyPoints) {
    // 合并垂直关键点
    for (int y = 0; y < height_; ++y) {
        for (const auto& keyPoint : verticalKeyPoints[y]) {
            for (size_t i = 0; i < subspaces_.size(); ++i) {
                if (isIntervalInSubspace(Interval(keyPoint.point.x, keyPoint.point.x), i)) {
                    subspaces_[i].keyPoints.insert(keyPoint);
                    break;
                }
            }
        }
    }

    // 合并水平关键点
    for (int y = 0; y < height_; ++y) {
        for (const auto& keyPoint : horizontalKeyPoints[y]) {
            for (size_t i = 0; i < subspaces_.size(); ++i) {
                if (isIntervalInSubspace(Interval(keyPoint.point.y, keyPoint.point.y), i)) {
                    subspaces_[i].keyPoints.insert(keyPoint);
                    break;
                }
            }
        }
    }
}

bool Preprocess2::isIntervalInSubspace(const Interval& interval, size_t subspaceIndex) const {
    for (const auto& subspaceInterval : subspaces_[subspaceIndex].verticalIntervals) {
        if (interval.start >= subspaceInterval.start && interval.end <= subspaceInterval.end) {
            return true;
        }
    }
    return false;
}

void Preprocess2::buildNeighbors() {
    // 为每个子空间初始化邻居集合
    for (size_t i = 0; i < subspaces_.size(); ++i) {
        subspaces_[i].neighbors.clear();
    }

    // 检查每对子空间是否相邻
    for (size_t i = 0; i < subspaces_.size(); ++i) {
        for (size_t j = i + 1; j < subspaces_.size(); ++j) {
            if (areSubspacesNeighbors(subspaces_[i], subspaces_[j])) {
                subspaces_[i].neighbors.insert(j);
                subspaces_[j].neighbors.insert(i);
            }
        }
    }
}

bool Preprocess2::areSubspacesNeighbors(const Subspace& s1, const Subspace& s2) const {
    if (s1.verticalIntervals.empty() || s2.verticalIntervals.empty()) {
        return false;
    }

    // 获取s1的第一个和最后一个区间
    const Interval& s1_first = s1.verticalIntervals.front();
    const Interval& s1_last = s1.verticalIntervals.back();
    
    // 获取s2的第一个和最后一个区间
    const Interval& s2_first = s2.verticalIntervals.front();
    const Interval& s2_last = s2.verticalIntervals.back();

    // 情况1：s1的第一个区间与s2的最后一个区间相邻
    if (s1_first.start == s2_last.end + 1) {
        // 检查两个区间是否有交集
        if (s1_first.start <= s2_last.end && s1_first.end >= s2_last.start) {
            return true;
                }
            }

    // 情况2：s1的最后一个区间与s2的第一个区间相邻
    if (s1_last.end + 1 == s2_first.start) {
        // 检查两个区间是否有交集
        if (s1_last.start <= s2_first.end && s1_last.end >= s2_first.start) {
            return true;
        }
    }

    return false;
}

void Preprocess2::getMaxMovableSpace(int y, std::vector<Interval>& intervals) const {
    intervals.clear();
    int start = 0;
    bool inInterval = false;

    for (int x = 0; x < width_; ++x) {
        bool isObstacle = !utils::isPassable(grid_, x, y);
        
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

void Preprocess2::getMaxMovableSpaceVertical(int x, std::vector<Interval>& intervals) const {
    intervals.clear();
    int start = 0;
    bool inInterval = false;

    for (int y = 0; y < height_; ++y) {
        bool isObstacle = !utils::isPassable(grid_, x, y);
        
        if (!isObstacle) {
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

void Preprocess2::updateTrend(int curr, int prev, Trend& mark, Trend& prevMark) const {
    if (curr < prev) {
        mark = Trend::INCREASING;
    } else if (curr > prev) {
        mark = Trend::DECREASING;
    } else {
        mark = prevMark;
    }
} 