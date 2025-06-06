#include "Preprocess.h"
#include <iostream>
#include <algorithm>
#include <queue>

Preprocess::Preprocess(const std::vector<std::vector<int>>& grid)
    : grid_(grid), width_(grid[0].size()), height_(grid.size()) {
    verticalIntervals_.resize(height_);
}

void Preprocess::preprocess() {
    // 初始化verticalIntervals_的大小

    // 收集所有关键点
    collectKeyPoints();

    // 构建key interval
    buildKeyIntervals();

    // 构建邻居关系
    buildNeighbors();
}

void Preprocess::collectKeyPoints() {
    std::vector<Interval> prevColumnIntervals;
    std::vector<Interval> currentColumnIntervals;
    // 垂直扫描收集关键点
    for (int y = 0; y < width_; ++y) {
        if(y == 0) {
            getMaxMovableSpace(y, height_, true, currentColumnIntervals);
        }
        std::vector<Interval> nextColumnIntervals;
        if(y + 1 < width_) {
            getMaxMovableSpace(y + 1, height_, true, nextColumnIntervals);
        }
        processVerticalScan(y, currentColumnIntervals, prevColumnIntervals, nextColumnIntervals);
        prevColumnIntervals = currentColumnIntervals;
        currentColumnIntervals = nextColumnIntervals;
    }

    std::vector<Interval> prevRowIntervals;
    std::vector<Interval> currentRowIntervals;
    // 水平扫描收集关键点
    for (int x = 0; x < height_; ++x) {
        if(x == 0) {
            getMaxMovableSpace(x, width_, false, currentRowIntervals);
        }
        std::vector<Interval> nextRowIntervals;
        if(x + 1 < height_) {
            getMaxMovableSpace(x + 1, width_, false, nextRowIntervals);
        }
        processHorizontalScan(x, currentRowIntervals, prevRowIntervals, nextRowIntervals);
        prevRowIntervals = currentRowIntervals;
        currentRowIntervals = nextRowIntervals;
    }
}



void Preprocess::processVerticalScan(int y, std::vector<Interval>& prevColumnIntervals,
                                   std::vector<Interval>& currentColumnIntervals,
                                   std::vector<Interval>& nextColumnIntervals) {
    if (currentColumnIntervals.empty()) return;

    updateIntervalTrends(prevColumnIntervals, currentColumnIntervals, true, y);
    updateIntervalTrends(currentColumnIntervals, nextColumnIntervals, true, y);

    processKeyPoint(currentColumnIntervals, nextColumnIntervals, true, y);
}

void Preprocess::processHorizontalScan(int x, std::vector<Interval>& prevRowIntervals,std::vector<Interval>& currentRowIntervals, std::vector<Interval>& nextRowIntervals) {
    if (currentRowIntervals.empty()) return;

    updateIntervalTrends(prevRowIntervals, currentRowIntervals, false, x);
    updateIntervalTrends(currentRowIntervals, nextRowIntervals, false, x);

    processKeyPoint(currentRowIntervals, nextRowIntervals, false, x);
}

void Preprocess::updateIntervalTrends(std::vector<Interval>& prevIntervals, 
                                    std::vector<Interval>& currentIntervals, bool isVertical, int fixed) {
    for (size_t i = 0; i < currentIntervals.size(); ++i) {
        // 找到与当前区间连通的前一列区间
        std::vector<Interval> connectedIntervals;
        for (const auto& prevInterval : prevIntervals) {
            if (currentIntervals[i].start <= prevInterval.end && 
                currentIntervals[i].end >= prevInterval.start) {
                connectedIntervals.push_back(prevInterval);
                if(isVertical) {
                    verticalIntervals_[fixed-1][prevInterval.start].rightNeighbors.insert(IntervalKey(fixed, currentIntervals[i].start, currentIntervals[i].end));
                    verticalIntervals_[fixed][currentIntervals[i].start].leftNeighbors.insert(IntervalKey(fixed-1, prevInterval.start, prevInterval.end));
                }
            }
        }

        if (!connectedIntervals.empty()) {
            // 由于区间是按顺序排列的，直接使用第一个和最后一个区间
            int minStart = connectedIntervals.front().start;
            int maxEnd = connectedIntervals.back().end;
            Trend prevStartTrend = connectedIntervals.front().start_mark;
            Trend prevEndTrend = connectedIntervals.back().end_mark;
            currentIntervals[i].prev_start_mark = prevStartTrend;
            currentIntervals[i].prev_end_mark = prevEndTrend;

            // 更新start趋势
            if (currentIntervals[i].start > minStart) {
                currentIntervals[i].start_mark = Trend::DECREASING;
            } else if (currentIntervals[i].start < minStart) {
                currentIntervals[i].start_mark = Trend::INCREASING;
            } else {
                currentIntervals[i].start_mark = prevStartTrend;
            }

            // 更新end趋势
            if (currentIntervals[i].end > maxEnd) {
                currentIntervals[i].end_mark = Trend::INCREASING;
            } else if (currentIntervals[i].end < maxEnd) {
                currentIntervals[i].end_mark = Trend::DECREASING;
            } else {
                currentIntervals[i].end_mark = prevEndTrend;
            }
        } else {
            // 如果没有连通区间，保持UNCHANGED
            currentIntervals[i].start_mark = Trend::UNCHANGED;
            currentIntervals[i].end_mark = Trend::UNCHANGED;
        }
    }
}

void Preprocess::processKeyPoint(std::vector<Interval>& currentIntervals, 
                                    std::vector<Interval>& nextIntervals, bool isVertical, int fixed) {
    for (size_t i = 0; i < currentIntervals.size(); ++i) {
        // 找到与当前区间连通的前一列区间
        std::vector<Interval> connectedIntervals;
        for (const auto& nextInterval : nextIntervals) {
            if (currentIntervals[i].start <= nextInterval.end && 
                currentIntervals[i].end >= nextInterval.start) {
                connectedIntervals.push_back(nextInterval);
            }
        }

        if (!connectedIntervals.empty()) {
            // 由于区间是按顺序排列的，直接使用第一个和最后一个区间
            int minStart = connectedIntervals.front().start;
            int maxEnd = connectedIntervals.back().end;
            Trend nextStartTrend = connectedIntervals.front().start_mark;
            Trend nextEndTrend = connectedIntervals.back().end_mark;

            if(currentIntervals[i].prev_start_mark == Trend::DECREASING && nextStartTrend == Trend::INCREASING) {
                if(isVertical) {
                    verticalIntervals_[fixed][currentIntervals[i].start].verticalKeyPoints.insert(KeyPoint(Vertex(currentIntervals[i].start,fixed), Direction::DOWN));
                } else {
                    for(VerticalInterval verticalInterval : verticalIntervals_[currentIntervals[i].start]) {
                        if(verticalInterval.start <= fixed && verticalInterval.end >= fixed) {
                            verticalInterval.horizontalKeyPoints.insert(KeyPoint(Vertex(fixed, currentIntervals[i].start), Direction::RIGHT));
                        }
                    }
                }
            } else if(currentIntervals[i].prev_end_mark == Trend::DECREASING && nextEndTrend == Trend::INCREASING) {
                if(isVertical) {
                    verticalIntervals_[fixed][currentIntervals[i].end].verticalKeyPoints.insert(KeyPoint(Vertex(currentIntervals[i].end,fixed), Direction::UP));
                } else {
                    for(VerticalInterval verticalInterval : verticalIntervals_[currentIntervals[i].end]) {
                        if(verticalInterval.start <= fixed && verticalInterval.end >= fixed) {
                            verticalInterval.horizontalKeyPoints.insert(KeyPoint(Vertex(fixed, currentIntervals[i].end), Direction::LEFT));
                        }
                    }
                }
            }
        } 
    }
}

void Preprocess::buildKeyIntervals() {
    // 遍历所有vertical intervals，找出包含关键点的interval
    for (int y = 0; y < height_; ++y) {
        for (const auto& interval : verticalIntervals_[y]) {
            if (!interval.horizontalKeyPoints.empty() || !interval.verticalKeyPoints.empty()) {
                // 创建key
                IntervalKey key{interval.y, interval.start, interval.end};
                // 插入到hashmap
                keyIntervals_[key] = interval;
            }
        }
    }
}

void Preprocess::buildNeighbors() {
    // 为每个key interval构建邻居关系
    for (auto& [key, interval] : keyIntervals_) {
        const auto& currentInterval = verticalIntervals_[key.y][key.start];
        processNeighborPair(key, currentInterval.leftNeighbors, -1, Direction::LEFT);
        processNeighborPair(key, currentInterval.rightNeighbors, 1, Direction::RIGHT);
    }
}

void Preprocess::updateNeighborRelation(VerticalInterval& currentInterval,
                                      const VerticalInterval& nextInterval,
                                      int y,
                                      int nextY,
                                      size_t nextIndex) {
    IntervalKey nextKey{nextInterval.y, nextInterval.start, nextInterval.end};
    IntervalKey currentKey{currentInterval.y, currentInterval.start, currentInterval.end};
    
    if (nextY < y) {
        currentInterval.leftNeighbors.insert(nextKey);
        verticalIntervals_[nextY][nextIndex].rightNeighbors.insert(currentKey);
    } else {
        currentInterval.rightNeighbors.insert(nextKey);
        verticalIntervals_[nextY][nextIndex].leftNeighbors.insert(currentKey);
    }
}

void Preprocess::processNeighborPair(const IntervalKey& key,
                                   const std::set<IntervalKey>& directNeighbors,
                                   int step,
                                   Direction dir) {
    const auto& currentInterval = verticalIntervals_[key.y][key.start];
    
    for(const auto& neighbor : directNeighbors) {
        std::vector<std::pair<IntervalKey, IntervalKey>> dirNeighborToNeighbor;
        for(int y = neighbor.y; (step < 0 ? y >= 0 : y < height_); y += step) {
            IntervalKey searchKey{y, neighbor.start, neighbor.end};
            if(findKeyInterval(searchKey)) {
                dirNeighborToNeighbor.emplace_back(std::make_pair(key, searchKey));
            }
        }
        for(int i = 0; i < dirNeighborToNeighbor.size(); i++) {
            keyIntervals_[key].neighbors.insert(dirNeighborToNeighbor[i].second);
            for(int j = i + 1; j < dirNeighborToNeighbor.size(); j++) {
                const auto& [directNeighbor1, keyIntervalNeighbor1] = dirNeighborToNeighbor[i];
                const auto& [directNeighbor2, keyIntervalNeighbor2] = dirNeighborToNeighbor[j];
                for(const auto& keyPoint : currentInterval.horizontalKeyPoints) {
                    if(keyPoint.direction == dir && 
                       keyPoint.point.x >= std::max(directNeighbor1.end, directNeighbor2.end) && 
                       keyPoint.point.x <= std::min(directNeighbor1.start, directNeighbor2.start)) {
                        processMustPassPoint(key, keyIntervalNeighbor1, keyIntervalNeighbor2, keyPoint);
                        break;
                    }
                }
            }
        }
    }
}

void Preprocess::processMustPassPoint(const IntervalKey& key,
                                    const IntervalKey& neighbor1,
                                    const IntervalKey& neighbor2,
                                    const KeyPoint& keyPoint) {
    keyIntervals_[neighbor1].neighborMustPassPoints[neighbor2][neighbor1] = keyPoint.point;
    keyIntervals_[neighbor2].neighborMustPassPoints[neighbor1][neighbor2] = keyPoint.point;
}

void Preprocess::getMaxMovableSpace(int fixed, int range, bool isVertical, std::vector<Interval>& intervals) const {
    intervals.clear();
    int start = 0;
    bool inInterval = false;

    for (int i = 0; i < range; ++i) {
        bool isObstacle = !utils::isPassable(grid_, isVertical ? fixed : i, isVertical ? i : fixed);
        
        if (!isObstacle) {
            if (!inInterval) {
                start = i;
                inInterval = true;
            }
        } else {
            if (inInterval) {
                intervals.emplace_back(start, i - 1);
                inInterval = false;
            }
        }
    }

    if (inInterval) {
        intervals.emplace_back(start, range - 1);
    }
}
