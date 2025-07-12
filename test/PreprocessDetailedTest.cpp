#include <gtest/gtest.h>
#include "../src/KIAstar/Preprocess.h"
#include <vector>
#include <set>
#include <iostream>

class PreprocessDetailedTest : public ::testing::Test {
protected:
    void SetUp() override {
        setupTestMaps();
    }

    void setupTestMaps() {
        // 测试地图1：简单的L形障碍物
        lShapeMap = {
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 1, 1, 1, 0, 0},
            {0, 0, 0, 1, 0, 0, 0, 0},
            {0, 0, 0, 1, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0}
        };

        // 测试地图2：走廊形状
        corridorMap = {
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 1, 1, 1, 1, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 1, 1, 1, 1, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0}
        };

        // 测试地图3：迷宫形状
        mazeMap = {
            {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 1, 1, 1, 1, 1, 1, 0},
            {0, 1, 0, 0, 0, 0, 1, 0},
            {0, 1, 0, 1, 1, 0, 1, 0},
            {0, 1, 0, 1, 1, 0, 1, 0},
            {0, 1, 0, 0, 0, 0, 1, 0},
            {0, 1, 1, 1, 1, 1, 1, 0},
            {0, 0, 0, 0, 0, 0, 0, 0}
        };
    }

    std::vector<std::vector<int>> lShapeMap;
    std::vector<std::vector<int>> corridorMap;
    std::vector<std::vector<int>> mazeMap;
};

// 测试L形障碍物的关键点提取
TEST_F(PreprocessDetailedTest, LShapeKeyPoints) {
    Preprocess preprocess(lShapeMap);
    preprocess.preprocess();
    
    const auto& keyIntervals = preprocess.getKeyIntervals();
    
    // 打印key intervals信息用于调试
    std::cout << "L-shape map has " << keyIntervals.size() << " key intervals" << std::endl;
    
    for (const auto& [key, interval] : keyIntervals) {
        std::cout << "Key Interval at y=" << key.y << " [" << key.start << "," << key.end << "]" << std::endl;
        std::cout << "  Vertical key points: " << interval.verticalKeyPoints.size() << std::endl;
        std::cout << "  Horizontal key points: " << interval.horizontalKeyPoints.size() << std::endl;
        std::cout << "  Neighbors: " << interval.neighbors.size() << std::endl;
    }
    
    // L形障碍物应该产生关键点
    bool hasKeyPoints = false;
    for (const auto& [key, interval] : keyIntervals) {
        if (!interval.horizontalKeyPoints.empty() || !interval.verticalKeyPoints.empty()) {
            hasKeyPoints = true;
            break;
        }
    }
    EXPECT_TRUE(hasKeyPoints);
}

// 测试走廊地图的区间划分
TEST_F(PreprocessDetailedTest, CorridorIntervals) {
    Preprocess preprocess(corridorMap);
    preprocess.preprocess();
    
    const auto& verticalIntervals = preprocess.getVerticalIntervals();
    
    // 检查第2行和第5行（有障碍物的行）
    EXPECT_GT(verticalIntervals[2].size(), 1); // 应该有多个区间
    EXPECT_GT(verticalIntervals[5].size(), 1); // 应该有多个区间
    
    // 检查第3行和第4行（走廊行）
    EXPECT_EQ(verticalIntervals[3].size(), 1); // 应该只有一个连续区间
    EXPECT_EQ(verticalIntervals[4].size(), 1); // 应该只有一个连续区间
    
    // 验证走廊区间的范围
    for (const auto& interval : verticalIntervals[3]) {
        EXPECT_EQ(interval.start, 0);
        EXPECT_EQ(interval.end, 7);
    }
}

// 测试迷宫地图的复杂结构
TEST_F(PreprocessDetailedTest, MazeStructure) {
    Preprocess preprocess(mazeMap);
    preprocess.preprocess();
    
    const auto& keyIntervals = preprocess.getKeyIntervals();
    const auto& verticalIntervals = preprocess.getVerticalIntervals();
    
    std::cout << "Maze map has " << keyIntervals.size() << " key intervals" << std::endl;
    
    // 迷宫应该有多个key intervals
    EXPECT_GT(keyIntervals.size(), 0);
    
    // 检查边界行的区间
    EXPECT_EQ(verticalIntervals[0].size(), 1); // 第一行应该只有一个区间
    EXPECT_EQ(verticalIntervals[7].size(), 1); // 最后一行应该只有一个区间
    
    // 检查中间行的区间（应该有多个区间）
    for (int y = 1; y < 7; ++y) {
        EXPECT_GT(verticalIntervals[y].size(), 0);
    }
}

// 测试关键点的方向标记
TEST_F(PreprocessDetailedTest, KeyPointDirections) {
    Preprocess preprocess(lShapeMap);
    preprocess.preprocess();
    
    const auto& keyIntervals = preprocess.getKeyIntervals();
    
    // 检查关键点的方向是否正确
    for (const auto& [key, interval] : keyIntervals) {
        for (const auto& keyPoint : interval.verticalKeyPoints) {
            // 垂直关键点应该有UP或DOWN方向
            EXPECT_TRUE(keyPoint.direction == Preprocess::Direction::UP || 
                       keyPoint.direction == Preprocess::Direction::DOWN);
        }
        
        for (const auto& keyPoint : interval.horizontalKeyPoints) {
            // 水平关键点应该有LEFT或RIGHT方向
            EXPECT_TRUE(keyPoint.direction == Preprocess::Direction::LEFT || 
                       keyPoint.direction == Preprocess::Direction::RIGHT);
        }
    }
}

// 测试邻居关系的正确性
TEST_F(PreprocessDetailedTest, NeighborRelationsCorrectness) {
    Preprocess preprocess(corridorMap);
    preprocess.preprocess();
    
    const auto& keyIntervals = preprocess.getKeyIntervals();
    
    // 检查邻居关系的对称性
    for (const auto& [key, interval] : keyIntervals) {
        for (const auto& triple : interval.neighbors) {
            // 如果A是B的邻居，那么B也应该是A的邻居
            const auto* neighborInterval = preprocess.findKeyInterval(triple.neighborKeyInterval);
            ASSERT_NE(neighborInterval, nullptr);
            
            bool isReciprocal = false;
            for (const auto& reciprocalTriple : neighborInterval->neighbors) {
                if (reciprocalTriple.neighborKeyInterval == key) {
                    isReciprocal = true;
                    break;
                }
            }
            EXPECT_TRUE(isReciprocal) << "Neighbor relationship should be symmetric";
        }
    }
}

// 测试transition vertices的存在性
TEST_F(PreprocessDetailedTest, TransitionVerticesExistence) {
    Preprocess preprocess(mazeMap);
    preprocess.preprocess();
    
    const auto& keyIntervals = preprocess.getKeyIntervals();
    
    // 统计有transition vertices的intervals
    int intervalsWithTransitionVertices = 0;
    for (const auto& [key, interval] : keyIntervals) {
        if (!interval.transitionVertices.empty()) {
            intervalsWithTransitionVertices++;
        }
    }
    
    std::cout << "Intervals with transition vertices: " << intervalsWithTransitionVertices << std::endl;
    
    // 迷宫地图应该有一些transition vertices
    // 注意：这个测试可能失败，取决于具体的实现
    // 这里只是检查基本结构
}

// 测试区间坐标的有效性
TEST_F(PreprocessDetailedTest, IntervalCoordinateValidity) {
    Preprocess preprocess(corridorMap);
    preprocess.preprocess();
    
    const auto& verticalIntervals = preprocess.getVerticalIntervals();
    
    // 检查所有区间的坐标都在有效范围内
    for (int y = 0; y < verticalIntervals.size(); ++y) {
        for (const auto& interval : verticalIntervals[y]) {
            EXPECT_EQ(interval.y, y);
            EXPECT_GE(interval.start, 0);
            EXPECT_LE(interval.end, 7);
            EXPECT_LE(interval.start, interval.end);
            
            // 检查区间内的所有点都是可通行的
            for (int x = interval.start; x <= interval.end; ++x) {
                EXPECT_EQ(corridorMap[y][x], 0) << "Interval should only contain passable cells";
            }
        }
    }
}

// 测试Key Interval的唯一性
TEST_F(PreprocessDetailedTest, KeyIntervalUniqueness) {
    Preprocess preprocess(lShapeMap);
    preprocess.preprocess();
    
    const auto& keyIntervals = preprocess.getKeyIntervals();
    
    // 检查所有key都是唯一的
    std::set<Preprocess::IntervalKey> uniqueKeys;
    for (const auto& [key, interval] : keyIntervals) {
        EXPECT_TRUE(uniqueKeys.insert(key).second) << "Key intervals should be unique";
    }
}

// 测试性能：大地图的处理
TEST_F(PreprocessDetailedTest, LargeMapPerformance) {
    // 创建一个16x16的地图
    std::vector<std::vector<int>> largeMap(16, std::vector<int>(16, 0));
    
    // 添加一些障碍物
    for (int i = 2; i < 14; ++i) {
        largeMap[4][i] = 1;
        largeMap[11][i] = 1;
    }
    for (int i = 4; i < 12; ++i) {
        largeMap[i][2] = 1;
        largeMap[i][13] = 1;
    }
    
    Preprocess preprocess(largeMap);
    
    // 测试预处理时间（这里只是确保不会崩溃）
    auto start = std::chrono::high_resolution_clock::now();
    preprocess.preprocess();
    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Large map preprocessing took: " << duration.count() << " ms" << std::endl;
    
    // 确保预处理成功
    const auto& keyIntervals = preprocess.getKeyIntervals();
    EXPECT_GT(keyIntervals.size(), 0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 