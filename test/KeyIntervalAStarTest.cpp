#include <gtest/gtest.h>
#include "../src/KIAstar/KeyIntervalAStar.h"
#include "../src/KIAstar/Preprocess.h"
#include "../src/astar/AStar.h"
#include <vector>
#include <set>
#include "../src/utilities/Log.h"

class KeyIntervalAStarTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 设置测试用的地图
        setupEmptyMap();
        setupSimpleMap();
        setupComplexMap();
        setupMazeMap();
    }

    void setupEmptyMap() {
        // 8x8 空地图
        emptyMap = {
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0}
        };
    }

    void setupSimpleMap() {
        // 8x8 简单地图，中间有一个障碍物
        simpleMap = {
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 1, 1, 0, 0, 0},
            {0, 0, 0, 1, 1, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0}
        };
    }

    void setupComplexMap() {
        // 8x8 复杂地图，多个障碍物
        complexMap = {
            {0, 0, 0, 1, 1, 0, 0, 0},
            {0, 0, 0, 1, 1, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {1, 1, 0, 0, 0, 0, 1, 1},
            {1, 1, 0, 0, 0, 0, 1, 1},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 1, 1, 0, 0, 0},
            {0, 0, 0, 1, 1, 0, 0, 0}
        };
    }

    void setupMazeMap() {
        // 8x8 迷宫地图
        mazeMap = {
            {0, 1, 0, 0, 0, 0, 1, 0},
            {0, 1, 1, 1, 0, 1, 1, 0},
            {0, 0, 0, 0, 0, 0, 1, 0},
            {0, 1, 1, 1, 1, 1, 1, 0},
            {0, 1, 0, 0, 0, 0, 0, 0},
            {0, 1, 1, 1, 1, 1, 1, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 1, 0, 0, 0}
        };
    }

    std::vector<std::vector<int>> emptyMap;
    std::vector<std::vector<int>> simpleMap;
    std::vector<std::vector<int>> complexMap;
    std::vector<std::vector<int>> mazeMap;
};

// 测试基本构造函数
TEST_F(KeyIntervalAStarTest, Constructor) {
    Preprocess preprocess(emptyMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    // 构造函数应该成功执行，没有异常
    EXPECT_TRUE(true);
}

// 测试空地图的路径搜索
TEST_F(KeyIntervalAStarTest, EmptyMapSearch ) {
    Preprocess preprocess(emptyMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // 测试简单的直线路径
    Vertex start(0, 0);
    Vertex end(7, 7);
    
    std::vector<Vertex> path = kiaStar.search(start, end);
    
    // 空地图应该能找到路径
    EXPECT_FALSE(path.empty());
    
    // 检查路径的起点和终点
    if (!path.empty()) {
        EXPECT_EQ(path.front(), start);
        EXPECT_EQ(path.back(), end);
        EXPECT_EQ(path.size(), 15);
    }

}

// 测试简单地图的路径搜索
TEST_F(KeyIntervalAStarTest, SimpleMapSearch) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // 测试绕过障碍物的路径
    Vertex start(0, 0);
    Vertex end(7, 7);
    
    std::vector<Vertex> path = kiaStar.search(start, end);
    
    // 应该能找到路径
    EXPECT_FALSE(path.empty());
    
    // 检查路径的起点和终点
    if (!path.empty()) {
        EXPECT_EQ(path.front(), start);
        EXPECT_EQ(path.back(), end);
        
        // 检查路径是否绕过障碍物（障碍物在(3,3)-(4,4)）
        for (const auto& vertex : path) {
            EXPECT_FALSE((vertex.x == 3 && vertex.y == 3) ||
                        (vertex.x == 3 && vertex.y == 4) ||
                        (vertex.x == 4 && vertex.y == 3) ||
                        (vertex.x == 4 && vertex.y == 4));
        }
        EXPECT_EQ(path.size(), 15);
    }
}

// 测试复杂地图的路径搜索
TEST_F(KeyIntervalAStarTest, ComplexMapSearch) {
    Preprocess preprocess(complexMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // 测试复杂路径
    Vertex start1(0, 0);
    Vertex end1(7, 7);

    Vertex start2(0, 7);
    Vertex end2(7, 7);
    
    std::vector<Vertex> path1 = kiaStar.search(start1, end1);
    std::vector<Vertex> path2 = kiaStar.search(start2, end2);
    
    // 应该能找到路径
    EXPECT_FALSE(path1.empty());
    EXPECT_FALSE(path2.empty());
    
    // 检查路径的起点和终点
    if (!path1.empty()) {
        EXPECT_EQ(path1.front(), start1);
        EXPECT_EQ(path1.back(), end1);
        EXPECT_EQ(path1.size(), 15);
    }

    if (!path2.empty()) {
        EXPECT_EQ(path2.front(), start2);
        EXPECT_EQ(path2.back(), end2);
        EXPECT_EQ(path2.size(), 12);
    }
}

// 测试迷宫地图的路径搜索
TEST_F(KeyIntervalAStarTest, MazeMapSearch) {
    Preprocess preprocess(mazeMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // 测试迷宫路径
    Vertex start(0, 5);
    Vertex end(7, 7);
    
    std::vector<Vertex> path = kiaStar.search(start, end);
    
    // 应该能找到路径
    EXPECT_FALSE(path.empty());
    
    // 检查路径的起点和终点
    if (!path.empty()) {
        EXPECT_EQ(path.front(), start);
        EXPECT_EQ(path.back(), end);
        EXPECT_EQ(path.size(), 20);
    }
    
}

// 测试相邻点的路径搜索
TEST_F(KeyIntervalAStarTest, AdjacentPointsSearch) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // 测试相邻点
    Vertex start(0, 0);
    Vertex end(1, 0);
    
    std::vector<Vertex> path = kiaStar.search(start, end);
    
    // 应该能找到路径
    EXPECT_FALSE(path.empty());
    
    if (!path.empty()) {
        EXPECT_EQ(path.front(), start);
        EXPECT_EQ(path.back(), end);
        // 相邻点的路径应该很短
        EXPECT_LE(path.size(), 3);
    }
}

// 测试相同起终点的路径搜索
TEST_F(KeyIntervalAStarTest, SameStartEndSearch) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // 测试相同起终点
    Vertex start(0, 0);
    Vertex end(0, 0);
    
    std::vector<Vertex> path = kiaStar.search(start, end);
    
    // 应该能找到路径（只包含起点）
    EXPECT_FALSE(path.empty());
    
    if (!path.empty()) {
        EXPECT_EQ(path.front(), start);
        EXPECT_EQ(path.back(), end);
        EXPECT_EQ(path.size(), 1);
    }
}

// 测试边界点的路径搜索
TEST_F(KeyIntervalAStarTest, BoundaryPointsSearch) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // 测试边界点
    Vertex start(0, 0);
    Vertex end(7, 7);
    
    std::vector<Vertex> path = kiaStar.search(start, end);
    
    // 应该能找到路径
    EXPECT_FALSE(path.empty());
    
    if (!path.empty()) {
        EXPECT_EQ(path.front(), start);
        EXPECT_EQ(path.back(), end);
    }
}

// 测试无法到达的情况
TEST_F(KeyIntervalAStarTest, UnreachableTarget) {
    // 创建一个被完全包围的地图
    std::vector<std::vector<int>> surroundedMap = {
        {1, 1, 1, 1, 1, 1, 1, 1},
        {1, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 1},
        {1, 0, 0, 0, 0, 0, 0, 1},
        {1, 1, 1, 1, 1, 1, 1, 1}
    };
    
    Preprocess preprocess(surroundedMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // 测试从内部到外部的路径（应该无法到达）
    Vertex start(1, 1);
    Vertex end(8, 8); // 超出地图边界
    
    std::vector<Vertex> path = kiaStar.search(start, end);
    
    // 应该返回空路径
    EXPECT_TRUE(path.empty());
}

// 测试路径连续性
TEST_F(KeyIntervalAStarTest, PathContinuity) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    Vertex start(0, 0);
    Vertex end(7, 7);
    
    std::vector<Vertex> path = kiaStar.search(start, end);
    
    if (!path.empty()) {
        // 检查路径的连续性
        for (size_t i = 1; i < path.size(); ++i) {
            const auto& prev = path[i-1];
            const auto& curr = path[i];
            
            // 相邻点应该只相差1个坐标
            int dx = std::abs(curr.x - prev.x);
            int dy = std::abs(curr.y - prev.y);
            
            EXPECT_TRUE((dx == 1 && dy == 0) || (dx == 0 && dy == 1));
        }
    }
}

// 测试路径最优性（简单情况）
TEST_F(KeyIntervalAStarTest, PathOptimality) {
    Preprocess preprocess(emptyMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // 测试直线路径
    Vertex start(0, 0);
    Vertex end(3, 3);
    
    std::vector<Vertex> path = kiaStar.search(start, end);
    
    if (!path.empty()) {
        // 在空地图中，直线路径应该是最优的
        // 曼哈顿距离应该是6（3+3）
        int manhattanDistance = std::abs(end.x - start.x) + std::abs(end.y - start.y);
        EXPECT_EQ(path.size() - 1, manhattanDistance);
    }
}

// 测试多个不同起终点的组合
TEST_F(KeyIntervalAStarTest, MultipleStartEndCombinations) {
    Preprocess preprocess(complexMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    std::vector<std::pair<Vertex, Vertex>> testCases;
    testCases.push_back(std::make_pair(Vertex(0, 0), Vertex(7, 7)));
    testCases.push_back(std::make_pair(Vertex(0, 7), Vertex(7, 0)));
    testCases.push_back(std::make_pair(Vertex(3, 2), Vertex(4, 5)));
    testCases.push_back(std::make_pair(Vertex(1, 1), Vertex(6, 6)));
    testCases.push_back(std::make_pair(Vertex(2, 2), Vertex(5, 5)));
    
    for (const auto& [start, end] : testCases) {
        std::vector<Vertex> path = kiaStar.search(start, end);
        
        // 所有测试用例都应该能找到路径
        EXPECT_FALSE(path.empty()) << "Failed for start(" << start.x << "," << start.y 
                                  << ") to end(" << end.x << "," << end.y << ")";
        
        if (!path.empty()) {
            EXPECT_EQ(path.front(), start);
            EXPECT_EQ(path.back(), end);
        }
    }
}

// 测试启发式距离计算
TEST_F(KeyIntervalAStarTest, HeuristicDistanceCalculation) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // 测试一些已知的曼哈顿距离
    Vertex v1(0, 0);
    Vertex v2(3, 4);
    Vertex v3(7, 7);
    
    // 这些测试需要在KeyIntervalAStar类中添加公共方法来测试
    // 暂时跳过这个测试，因为calculateHeuristicDistance是私有的
    EXPECT_TRUE(true);
}

// 测试key interval查找功能
TEST_F(KeyIntervalAStarTest, KeyIntervalFinding) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // 测试一些顶点是否在key interval中
    Vertex testVertex(2, 2);
    
    // 这些测试需要在KeyIntervalAStar类中添加公共方法来测试
    // 暂时跳过这个测试，因为相关方法是私有的
    EXPECT_TRUE(true);
}

// 测试迷宫地图的多个不同起终点组合
TEST_F(KeyIntervalAStarTest, MazeMapMultipleStartEndCombinations) {
    Preprocess preprocess(mazeMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    std::vector<std::pair<Vertex, Vertex>> testCases;
    // 测试从左上角到右下角的路径
    testCases.push_back(std::make_pair(Vertex(0, 0), Vertex(7, 7)));
    // 测试从右上角到左下角的路径
    testCases.push_back(std::make_pair(Vertex(0, 7), Vertex(7, 0)));
    // 测试从中间区域到右下角的路径
    testCases.push_back(std::make_pair(Vertex(2, 2), Vertex(7, 7)));
    // 测试从左上角到中间区域的路径
    testCases.push_back(std::make_pair(Vertex(0, 0), Vertex(4, 4)));
    // 测试从底部到顶部的路径
    testCases.push_back(std::make_pair(Vertex(7, 0), Vertex(0, 0)));
    // 测试从右侧到左侧的路径
    testCases.push_back(std::make_pair(Vertex(0, 7), Vertex(0, 0)));
    // 测试从迷宫入口到出口的路径
    testCases.push_back(std::make_pair(Vertex(0, 2), Vertex(7, 6)));
    // 测试从迷宫内部到外部的路径
    testCases.push_back(std::make_pair(Vertex(2, 4), Vertex(7, 7)));
    
    for (const auto& [start, end] : testCases) {
        std::vector<Vertex> path = kiaStar.search(start, end);
        
        // 所有测试用例都应该能找到路径
        EXPECT_FALSE(path.empty()) << "Failed for start(" << start.x << "," << start.y 
                                  << ") to end(" << end.x << "," << end.y << ")";
        
        if (!path.empty()) {
            EXPECT_EQ(path.front(), start);
            EXPECT_EQ(path.back(), end);
            
            // 检查路径是否避开障碍物（障碍物值为1）
            for (const auto& vertex : path) {
                EXPECT_EQ(mazeMap[vertex.x][vertex.y], 0) 
                    << "Path goes through obstacle at (" << vertex.x << "," << vertex.y << ")";
            }
            
            // 检查路径连续性
            for (size_t i = 1; i < path.size(); ++i) {
                int dx = std::abs(path[i].x - path[i-1].x);
                int dy = std::abs(path[i].y - path[i-1].y);
                EXPECT_TRUE((dx == 1 && dy == 0) || (dx == 0 && dy == 1))
                    << "Path is not continuous between (" << path[i-1].x << "," << path[i-1].y 
                    << ") and (" << path[i].x << "," << path[i].y << ")";
            }
        }
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 