#pragma once

#include <vector>
#include <set>
#include <map>
#include "../basic/Vertex.h"
#include "../utilities/GridUtility.h"

class Preprocess1 {

public:
    enum class Trend {
        UNCHANGED = 0,
        DECREASING = 1,
        INCREASING = 2
    };

    struct ClusterInterval {
        int x1;
        int x2;
        Trend x1_mark;      // 趋势标记
        Trend x2_mark;      // 趋势标记

        ClusterInterval(int x1, int x2) : 
            x1(x1), x2(x2), 
            x1_mark(Trend::UNCHANGED), x2_mark(Trend::UNCHANGED) {}
    };

    struct ObstacleCluster {
        std::set<Vertex> cluster;
        std::set<Vertex> keyPoints;
    };

    // 构造函数
    Preprocess1(const std::vector<std::vector<int>>& grid);

    // 获取所有带有关键点的障碍物集群
    const std::vector<ObstacleCluster>& getClusters() const;

    // 预处理所有障碍物集群
    void preprocess();

private:    
    // 地图数据
    const std::vector<std::vector<int>>& grid_;
    // 地图尺寸
    int width_;
    int height_;
    // 预处理后的障碍物集群
    std::vector<ObstacleCluster> clusters_;
    
    // 扫描所有障碍物集群并找到关键点
    std::vector<ObstacleCluster> scanClusters() const;

    // 使用深度优先搜索来找到所有相连的障碍物
    void dfs(int x, int y, std::set<Vertex>& cluster, std::vector<std::vector<bool>>& visited) const;

    // 水平扫描
    void horizontalScan(const std::set<Vertex>& cluster, std::set<Vertex>& keyPoints) const;

    // 水平方向扫描
    void directionalHorizontalScan(const std::set<Vertex>& cluster, std::set<Vertex>& keyPoints,
                                 int startY, int endY) const;

    // 垂直扫描
    void verticalScan(const std::set<Vertex>& cluster, std::set<Vertex>& keyPoints) const;

    // 垂直方向扫描
    void directionalVerticalScan(const std::set<Vertex>& cluster, std::set<Vertex>& keyPoints,
                               int startX, int endX) const;

    // 获取一列的最大可移动空间
    void getMaxMovableSpace(int y, const std::set<Vertex>& cluster, 
                           std::vector<ClusterInterval>& intervals) const;

    // 获取一行的最大可移动空间
    void getMaxMovableSpaceVertical(int x, const std::set<Vertex>& cluster,
                                  std::vector<ClusterInterval>& intervals) const;

    // 更新趋势标记
    void updateTrend(int curr, int prev, Trend& mark, Trend& prevMark) const;

    // 检查趋势变化并记录关键点
    void checkTrendChange(const ClusterInterval& prev, const ClusterInterval& curr, 
                         int x, int y, std::set<Vertex>& keyPoints) const;


}; 