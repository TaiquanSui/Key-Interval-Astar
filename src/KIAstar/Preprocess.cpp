#include "Preprocess.h"
#include <iostream>
#include <algorithm>
#include <queue>
#include "../utilities/Log.h"
#include <chrono>

Preprocess::Preprocess(const std::vector<std::vector<int>>& grid)
    : grid_(grid), width_(grid[0].size()), height_(grid.size()) {
}

void Preprocess::preprocess() {
    auto start = std::chrono::steady_clock::now();
    // 处理垂直扫描
    processScan(true);
    // 处理水平扫描
    processScan(false);
    
    // 构建key interval
    buildKeyIntervals();
    // 构建邻居关系
    buildNeighbors();
    auto end = std::chrono::steady_clock::now();
    preprocess_time_ = std::chrono::duration<double>(end - start).count();
    //logger::log_info("preprocess done, time: " + std::to_string(preprocess_time_) + "s");
}

void Preprocess::processScan(bool isVertical) {
    //logger::log_info(isVertical ? "start vertical scan" : "start horizontal scan");
    
    std::vector<Interval> prevIntervals;
    std::vector<Interval> currentIntervals;
    
    int scanRange = isVertical ? width_ : height_;
    int fixedRange = isVertical ? height_ : width_;
    
    // 扫描收集关键点
    for (int fixed = 0; fixed < scanRange; ++fixed) {
        // 获取当前行/列的区间
        if(fixed == 0) {
            getMaxMovableSpace(fixed, fixedRange, isVertical, currentIntervals);
        }
        
        // 获取下一行/列的区间
        std::vector<Interval> nextIntervals;
        if(fixed + 1 < scanRange) {
            getMaxMovableSpace(fixed + 1, fixedRange, isVertical, nextIntervals);
        }
        
        // 处理当前行/列
        processCurrentIntervals(prevIntervals, currentIntervals, nextIntervals, isVertical, fixed);
        
        // 更新区间
        prevIntervals = currentIntervals;
        currentIntervals = nextIntervals;
    }
    
    //logger::log_info(isVertical ? "end vertical scan" : "end horizontal scan");
}

void Preprocess::processCurrentIntervals(std::vector<Interval>& prevIntervals, 
                                    std::vector<Interval>& currentIntervals, 
                                    std::vector<Interval>& nextIntervals, bool isVertical, int fixed) {
                                        
    for (size_t i = 0; i < currentIntervals.size(); ++i) {
        // 找到与当前区间连通的前一列区间
        if(isVertical) {
            verticalIntervals_[fixed][currentIntervals[i].start] = VerticalInterval(fixed, currentIntervals[i].start, currentIntervals[i].end);
            //logger::log_info("create verticalInterval: [" + std::to_string(verticalIntervals_[fixed][currentIntervals[i].start].y) + ", " + std::to_string(verticalIntervals_[fixed][currentIntervals[i].start].start) + ", " + std::to_string(verticalIntervals_[fixed][currentIntervals[i].start].end) + "]");
        }

        std::vector<Interval> prevConnectedIntervals;
        for (const auto& prevInterval : prevIntervals) {
            if (currentIntervals[i].start <= prevInterval.end && 
                currentIntervals[i].end >= prevInterval.start) {
                prevConnectedIntervals.push_back(prevInterval);
                if(isVertical) {
                    // 添加邻居关系（只在不是第一列时）
                    if (verticalIntervals_.find(fixed-1) == verticalIntervals_.end() || 
                        verticalIntervals_[fixed-1].find(prevInterval.start) == verticalIntervals_[fixed-1].end()) {
                            continue;
                    }
                    //logger::log_info("add leftNeighbor: [" + std::to_string(fixed-1) + ", " + std::to_string(prevInterval.start) + ", " + std::to_string(prevInterval.end) + "]");
                    //logger::log_info("add rightNeighbor: [" + std::to_string(fixed) + ", " + std::to_string(currentIntervals[i].start) + ", " + std::to_string(currentIntervals[i].end) + "]");
                    verticalIntervals_[fixed-1][prevInterval.start].rightNeighbors.push_back(IntervalKey{fixed, currentIntervals[i].start, currentIntervals[i].end});
                    verticalIntervals_[fixed][currentIntervals[i].start].leftNeighbors.push_back(IntervalKey{fixed-1, prevInterval.start, prevInterval.end});
                    
                }
            }
        }

        std::vector<Interval> nextConnectedIntervals;
        for (const auto& nextInterval : nextIntervals) {
            if (currentIntervals[i].start <= nextInterval.end && 
                currentIntervals[i].end >= nextInterval.start) {
                nextConnectedIntervals.push_back(nextInterval);
            }
        }

        //logger::log_info("prevConnectedIntervals: ");
        for(const auto& interval : prevConnectedIntervals) {
            //logger::log_info("["+ std::to_string(fixed-1) + "," + std::to_string(interval.start) + "," + std::to_string(interval.end) + "]");
        }

        //logger::log_info("nextConnectedIntervals: ");
        for(const auto& interval : nextConnectedIntervals) {
            //logger::log_info("["+ std::to_string(fixed+1) + "," + std::to_string(interval.start) + "," + std::to_string(interval.end) + "]");
        }

        if (!prevConnectedIntervals.empty()) {
            // 由于区间是按顺序排列的，直接使用第一个和最后一个区间
            int minStart = prevConnectedIntervals.front().start;
            int maxEnd = prevConnectedIntervals.back().end;
            Trend prevStartTrend = prevConnectedIntervals.front().start_mark;
            Trend prevEndTrend = prevConnectedIntervals.back().end_mark;

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


        if (!nextConnectedIntervals.empty()) {
            int minStart = nextConnectedIntervals.front().start;
            int maxEnd = nextConnectedIntervals.back().end;

            //logger::log_info("currentIntervals[i].start_mark: " + std::string(trendToString(currentIntervals[i].start_mark)));
            //logger::log_info("currentIntervals[i].end_mark: " + std::string(trendToString(currentIntervals[i].end_mark)));
            //logger::log_info("currentIntervals[i].start: " + std::to_string(currentIntervals[i].start) + ", minStart: " + std::to_string(minStart));
            //logger::log_info("currentIntervals[i].end: " + std::to_string(currentIntervals[i].end) + ", maxEnd: " + std::to_string(maxEnd));

            if(currentIntervals[i].start_mark == Trend::DECREASING && currentIntervals[i].start > minStart) {
                if(isVertical) {
                    // 只添加关键点，VerticalInterval已经在updateIntervalTrends中创建
                    Vertex keyPoint(currentIntervals[i].start,fixed);
                    //logger::log_info("processKeyPoint: keyPoint: " + std::to_string(keyPoint.x) + ", " + std::to_string(keyPoint.y));
                    // 确保verticalInterval存在
                    if (verticalIntervals_.find(fixed) == verticalIntervals_.end() || 
                        verticalIntervals_[fixed].find(currentIntervals[i].start) == verticalIntervals_[fixed].end()) {
                        verticalIntervals_[fixed][currentIntervals[i].start] = VerticalInterval(fixed, currentIntervals[i].start, currentIntervals[i].end);
                    }
                    verticalIntervals_[fixed][currentIntervals[i].start].downVertex = keyPoint;
                } else {
                    auto& yMap = verticalIntervals_[currentIntervals[i].start];
                    for(auto& [start, verticalInterval] : yMap) {
                        if(verticalInterval.start <= fixed && verticalInterval.end >= fixed) {
                            Vertex keyPoint(fixed, currentIntervals[i].start);
                            //logger::log_info("processKeyPoint: keyPoint: " + std::to_string(keyPoint.x) + ", " + std::to_string(keyPoint.y));
                            verticalInterval.horizontalKeyPoints.push_back(KeyPoint(keyPoint, Direction::RIGHT));
                        }
                    }
                }
            } 
            
            if(currentIntervals[i].end_mark == Trend::DECREASING && currentIntervals[i].end < maxEnd) {
                if(isVertical) {
                    // 只添加关键点，VerticalInterval已经在updateIntervalTrends中创建
                    Vertex keyPoint(currentIntervals[i].end,fixed);
                    //logger::log_info("processKeyPoint: keyPoint: " + std::to_string(keyPoint.x) + ", " + std::to_string(keyPoint.y));
                    verticalIntervals_[fixed][currentIntervals[i].start].upVertex = keyPoint;
                } else {
                    auto& yMap = verticalIntervals_[currentIntervals[i].end];
                    for(auto& [start, verticalInterval] : yMap) {
                        if(verticalInterval.start <= fixed && verticalInterval.end >= fixed) {
                            Vertex keyPoint(fixed, currentIntervals[i].end);
                            //logger::log_info("processKeyPoint: keyPoint: " + std::to_string(keyPoint.x) + ", " + std::to_string(keyPoint.y));
                            verticalInterval.horizontalKeyPoints.push_back(KeyPoint(keyPoint, Direction::LEFT));
                        }
                    }
                }
            }
        }
    }
}


void Preprocess::buildKeyIntervals() {
    // 遍历所有vertical intervals，找出包含关键点的interval
    for (const auto& [y, yMap] : verticalIntervals_) {
        for (const auto& [start, interval] : yMap) {
            //logger::log_info("verticalInterval: y: " + std::to_string(interval.y) + ", start: " + std::to_string(interval.start) + ", end: " + std::to_string(interval.end));
            if (interval.upVertex.has_value() || interval.downVertex.has_value() || !interval.horizontalKeyPoints.empty()) {
                // 创建key
                IntervalKey key{interval.y, interval.start, interval.end};
                // 创建KeyInterval并插入到hashmap（传递verticalIntervals_中实际对象的指针）
                keyIntervals_[key] = KeyInterval(&verticalIntervals_[y][start]);
                //logger::log_info("buildKeyIntervals: [" + std::to_string(key.y) + ", " + std::to_string(key.start) + ", " + std::to_string(key.end) + "]");
                for(const auto& keyPoint : interval.horizontalKeyPoints) {
                    //logger::log_info("horizontalKeyPoint: " + std::to_string(keyPoint.point.x) + ", " + std::to_string(keyPoint.point.y));
                }
                if(interval.upVertex.has_value()) {
                    //logger::log_info("upVertex: " + std::to_string(interval.upVertex->x) + ", " + std::to_string(interval.upVertex->y));
                }
                if(interval.downVertex.has_value()) {
                    //logger::log_info("downVertex: " + std::to_string(interval.downVertex->x) + ", " + std::to_string(interval.downVertex->y));
                }
            }
        }
    }
}

void Preprocess::buildNeighbors() {
    // 为每个key interval构建邻居关系
    for (auto& [key, interval] : keyIntervals_) {
        // 检查verticalInterval是否存在
        if (verticalIntervals_.find(key.y) == verticalIntervals_.end() || 
            verticalIntervals_[key.y].find(key.start) == verticalIntervals_[key.y].end()) {
            //logger::log_info("Warning: verticalInterval not found for key: y=" + std::to_string(key.y) + ", start=" + std::to_string(key.start));
            continue;
        }
        const auto& currentInterval = verticalIntervals_[key.y][key.start];
        //logger::log_info("buildNeighbors of [" + std::to_string(key.y) + ", " + std::to_string(key.start) + ", " + std::to_string(key.end) + "]");
        for(const auto& leftNeighbor : currentInterval.leftNeighbors) {
            //logger::log_info("leftNeighbor: [" + std::to_string(leftNeighbor.y) + ", " + std::to_string(leftNeighbor.start) + ", " + std::to_string(leftNeighbor.end) + "]");
        }
        for(const auto& rightNeighbor : currentInterval.rightNeighbors) {
            //logger::log_info("rightNeighbor: [" + std::to_string(rightNeighbor.y) + ", " + std::to_string(rightNeighbor.start) + ", " + std::to_string(rightNeighbor.end) + "]");
        }
        processNeighborPair(key, currentInterval.leftNeighbors, Direction::RIGHT);
        processNeighborPair(key, currentInterval.rightNeighbors, Direction::LEFT);

    }
}


void Preprocess::processNeighborPair(const IntervalKey& key,
                                   const std::vector<IntervalKey>& directNeighbors,
                                   Direction dir) {
    // 检查verticalInterval是否存在
    if (verticalIntervals_.find(key.y) == verticalIntervals_.end() || 
        verticalIntervals_[key.y].find(key.start) == verticalIntervals_[key.y].end()) {
        //logger::log_info("Warning: verticalInterval not found in processNeighborPair for key: y=" + std::to_string(key.y) + ", start=" + std::to_string(key.start));
        return;
    }
    //logger::log_info("processNeighborPair:[" + std::to_string(key.y) + ", " + std::to_string(key.start) + ", " + std::to_string(key.end) + "]");
    //logger::log_info("dir: " + std::string(dir == Direction::RIGHT ? "RIGHT" : "LEFT"));
    const auto& currentInterval = verticalIntervals_[key.y][key.start];

    // 为每个直接邻居查找对应的关键区间邻居
    for (const auto& dirNeighbor : directNeighbors) {
        // 首先检查neighbor本身是否是key interval
        if (keyIntervals_.find(dirNeighbor) != keyIntervals_.end()) {
            // 如果neighbor本身就是key interval，创建三元组
            NeighborTriple triple(dirNeighbor, key, dirNeighbor);
            keyIntervals_[key].neighbors.push_back(triple);
            //logger::log_info("direct key interval neighbor: [" + std::to_string(dirNeighbor.y) + ", " + std::to_string(dirNeighbor.start) + ", " + std::to_string(dirNeighbor.end) + "]");
            continue;
        }
        
        // 检查dirNeighbor是否存在
        if (verticalIntervals_.find(dirNeighbor.y) == verticalIntervals_.end() || 
            verticalIntervals_[dirNeighbor.y].find(dirNeighbor.start) == verticalIntervals_[dirNeighbor.y].end()) {
            //logger::log_info("Warning: dirNeighbor not found: [" + std::to_string(dirNeighbor.y) + ", " + std::to_string(dirNeighbor.start) + ", " + std::to_string(dirNeighbor.end) + "]");
            continue;
        }
        
        VerticalInterval verticalInterval = verticalIntervals_[dirNeighbor.y][dirNeighbor.start];
        std::vector<IntervalKey> intervalsBetween = {dirNeighbor};
        
        // 遍历邻居直到找到关键区间
        while (true) {
            //logger::log_info("Iterate until find key interval");
            // 根据当前verticalInterval选择要遍历的邻居集合
            const auto& neighborSet = (dir == Direction::RIGHT) ? 
                verticalInterval.leftNeighbors : verticalInterval.rightNeighbors;

            if (neighborSet.empty()) {
                for(const auto& interval : intervalsBetween) {
                    if(dir == Direction::RIGHT) {
                        verticalIntervals_[interval.y][interval.start].rightKeyInterval = std::make_pair(key, intervalsBetween.at(0));
                    } else {
                        verticalIntervals_[interval.y][interval.start].leftKeyInterval = std::make_pair(key, intervalsBetween.at(0));
                    }
                }
                //logger::log_info("Warning: No more neighbors found for dirNeighbor: [" + std::to_string(dirNeighbor.y) + ", " + std::to_string(dirNeighbor.start) + ", " + std::to_string(dirNeighbor.end) + "]");
                break;  // 没有更多邻居，退出循环
            }
            
            const auto& nextNeighbor = neighborSet.front();
            if (keyIntervals_.find(nextNeighbor) != keyIntervals_.end()) {
                // 找到关键区间，创建三元组
                NeighborTriple triple(dirNeighbor, intervalsBetween.back(), nextNeighbor);
                keyIntervals_[key].neighbors.push_back(triple);
                for(const auto& interval : intervalsBetween) {
                    if(dir == Direction::RIGHT) {
                        verticalIntervals_[interval.y][interval.start].rightKeyInterval = std::make_pair(key, intervalsBetween.at(0));
                        verticalIntervals_[interval.y][interval.start].leftKeyInterval = std::make_pair(nextNeighbor, intervalsBetween.back());
                    } else {
                        verticalIntervals_[interval.y][interval.start].rightKeyInterval = std::make_pair(nextNeighbor, intervalsBetween.back());
                        verticalIntervals_[interval.y][interval.start].leftKeyInterval = std::make_pair(key, intervalsBetween.at(0));
                    }
                }
                //logger::log_info("keyIntervalNeighbor: [" + std::to_string(nextNeighbor.y) + ", " + std::to_string(nextNeighbor.start) + ", " + std::to_string(nextNeighbor.end) + "]");
                //logger::log_info("currentDirectNeighbor: [" + std::to_string(dirNeighbor.y) + ", " + std::to_string(dirNeighbor.start) + ", " + std::to_string(dirNeighbor.end) + "]");
                //logger::log_info("neighborDirectNeighbor: [" + std::to_string(intervalsBetween.back().y) + ", " + std::to_string(intervalsBetween.back().start) + ", " + std::to_string(intervalsBetween.back().end) + "]");
                break;  // 找到关键区间，退出循环
            }
            // 继续遍历，更新verticalInterval和pathDirectNeighbor
            intervalsBetween.push_back(nextNeighbor);
            verticalInterval = verticalIntervals_[nextNeighbor.y][nextNeighbor.start];
        }
    }

    processTransitionVertices(key, currentInterval, directNeighbors, dir);
}

void Preprocess::processTransitionVertices(const IntervalKey& key,
                                            const VerticalInterval& currentInterval,
                                            const std::vector<IntervalKey>& directNeighbors,
                                            Direction dir) {
    // 记录直接邻居之间的transition vertices
    // 处理直接邻居之间的transition vertices
    for (const auto& dirNeighbor1 : directNeighbors) {
        for (const auto& dirNeighbor2 : directNeighbors) {
            if (dirNeighbor1 == dirNeighbor2) continue;
            
            // 检查关键点是否可以作为过渡顶点
            for (const auto& keyPoint : currentInterval.horizontalKeyPoints) {
                if (keyPoint.direction == dir && 
                    keyPoint.point.x >= std::min(dirNeighbor1.end, dirNeighbor2.end) && 
                    keyPoint.point.x <= std::max(dirNeighbor1.start, dirNeighbor2.start)) {
                    //logger::log_info("processTransitionVertices:");
                    //logger::log_info("directNeighbor1: [" + std::to_string(dirNeighbor1.y) + ", " + std::to_string(dirNeighbor1.start) + ", " + std::to_string(dirNeighbor1.end) + "]");
                    //logger::log_info("directNeighbor2: [" + std::to_string(dirNeighbor2.y) + ", " + std::to_string(dirNeighbor2.start) + ", " + std::to_string(dirNeighbor2.end) + "]");
                    //logger::log_info("keyPoint: [" + std::to_string(keyPoint.point.x) + ", " + std::to_string(keyPoint.point.y) + "]");
                    keyIntervals_[key].transitionVertices[dirNeighbor1][dirNeighbor2] = keyPoint.point;
                    keyIntervals_[key].transitionVertices[dirNeighbor2][dirNeighbor1] = keyPoint.point;
                    break;
                }
            }
        }
    }
    
}

void Preprocess::getMaxMovableSpace(int fixed, int range, bool isVertical, std::vector<Interval>& intervals) const {
    intervals.clear();
    int start = 0;
    bool inInterval = false;

    for (int i = 0; i < range; ++i) {
        bool isObstacle = !utils::isPassable(grid_, isVertical ? i : fixed, isVertical ? fixed : i);
        
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

std::optional<Preprocess::VerticalInterval> Preprocess::findContainingVerticalInterval(const Vertex& vertex) const {
    const auto& verticalIntervals = getVerticalIntervals();
    auto yIt = verticalIntervals.find(vertex.y);
    if (yIt == verticalIntervals.end()) {
        return std::nullopt;
    }
    for (const auto& [_, interval] : yIt->second) {
        if (vertex.x >= interval.start && vertex.x <= interval.end) {
            return interval;
        }
    }
    return std::nullopt;
}

bool Preprocess::isVertexInKeyInterval(const Vertex& vertex, const IntervalKey& key) {
    return vertex.y == key.y && vertex.x >= key.start && vertex.x <= key.end;
}

std::optional<Vertex> Preprocess::findTransitionVertex(const IntervalKey& key,
                                                      const IntervalKey& directNeighbor1,
                                                      const IntervalKey& directNeighbor2) const {
    auto keyIntervalOpt = findKeyInterval(key);
    if (!keyIntervalOpt.has_value()) {
        return std::nullopt;
    }

    auto keyInterval = keyIntervalOpt.value();
    
    auto it = keyInterval.transitionVertices.find(directNeighbor1);
    if (it != keyInterval.transitionVertices.end()) {
        auto it2 = it->second.find(directNeighbor2);
        if (it2 != it->second.end()) {
            return it2->second;
        }
    }
    
    return std::nullopt;
}

size_t Preprocess::getMemoryUsage() const {
    size_t total_memory = 0;
    
    // 计算grid_占用的内存
    total_memory += grid_.size() * grid_[0].size() * sizeof(int);
    
    // 计算verticalIntervals_占用的内存
    for (const auto& [y, yMap] : verticalIntervals_) {
        for (const auto& [start, interval] : yMap) {
            // VerticalInterval结构体大小
            total_memory += sizeof(VerticalInterval);
            // 动态分配的内存
            total_memory += interval.horizontalKeyPoints.size() * sizeof(KeyPoint);
            total_memory += interval.leftNeighbors.size() * sizeof(IntervalKey);
            total_memory += interval.rightNeighbors.size() * sizeof(IntervalKey);
        }
    }
    
    // 计算keyIntervals_占用的内存
    for (const auto& [key, keyInterval] : keyIntervals_) {
        // KeyInterval结构体大小
        total_memory += sizeof(KeyInterval);
        // 动态分配的内存
        total_memory += keyInterval.neighbors.size() * sizeof(NeighborTriple);
        // transitionVertices占用的内存
        for (const auto& [key1, innerMap] : keyInterval.transitionVertices) {
            total_memory += innerMap.size() * sizeof(std::pair<IntervalKey, Vertex>);
        }
    }
    
    return total_memory;
}
