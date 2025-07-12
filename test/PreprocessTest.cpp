#include <gtest/gtest.h>
#include "../src/KIAstar/Preprocess.h"
#include <vector>
#include <set>
#include "../src/utilities/Log.h"

class PreprocessTest : public ::testing::Test {
protected:
    void SetUp() override {
        // 设置测试用的地图
        setupEmptyMap();
        setupSimpleMap();
        setupComplexMap();
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

    std::vector<std::vector<int>> emptyMap;
    std::vector<std::vector<int>> simpleMap;
    std::vector<std::vector<int>> complexMap;
};

// 测试基本构造函数和地图尺寸
TEST_F(PreprocessTest, ConstructorAndMapSize) {
    Preprocess preprocess(emptyMap);
    
    EXPECT_EQ(preprocess.getHeight(), 8);
    EXPECT_EQ(preprocess.getWidth(), 8);
    
    const auto& grid = preprocess.getGrid();
    EXPECT_EQ(grid.size(), 8);
    EXPECT_EQ(grid[0].size(), 8);
}

// 测试空地图的预处理
TEST_F(PreprocessTest, EmptyMapPreprocess) {
    Preprocess preprocess(emptyMap);
    preprocess.preprocess();
    
    // 空地图应该没有key intervals
    const auto& verticalIntervals = preprocess.getVerticalIntervals();
    const auto& keyIntervals = preprocess.getKeyIntervals();
    EXPECT_EQ(verticalIntervals.size(), 8);
    EXPECT_EQ(keyIntervals.size(), 0);
}

// 测试简单地图的预处理
TEST_F(PreprocessTest, SimpleMapPreprocess) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    const auto& keyIntervals = preprocess.getKeyIntervals();
    // 简单地图应该有一些key intervals（在障碍物边界处）
    EXPECT_EQ(keyIntervals.size(), 4);
}

// 测试findKeyInterval方法
TEST_F(PreprocessTest, FindKeyInterval) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    const auto& keyIntervals = preprocess.getKeyIntervals();
    
    // 测试查找存在的key interval
    for (const auto& [key, interval] : keyIntervals) {
        const auto* found = preprocess.findKeyInterval(key);
        EXPECT_NE(found, nullptr);
        EXPECT_EQ(found->getY(), interval.getY());
        EXPECT_EQ(found->getStart(), interval.getStart());
        EXPECT_EQ(found->getEnd(), interval.getEnd());
    }
    
    // 测试查找不存在的key interval
    Preprocess::IntervalKey nonExistentKey{999, 999, 999};
    const auto* notFound = preprocess.findKeyInterval(nonExistentKey);
    EXPECT_EQ(notFound, nullptr);
}


// 测试关键点提取
TEST_F(PreprocessTest, KeyPointExtraction) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    const auto* interval1 = preprocess.findKeyInterval({4,0,2});
    const auto* interval2 = preprocess.findKeyInterval({4,5,7});
    const auto* interval3 = preprocess.findKeyInterval({2,0,7});
    const auto* interval4 = preprocess.findKeyInterval({5,0,7});
    
    // 只测试存在的interval
    if (interval1 != nullptr) {
        EXPECT_GT(interval1->getVerticalKeyPoints().size(), 0);
    }else{
        EXPECT_TRUE(false);
    }
    if (interval2 != nullptr) {
        EXPECT_GT(interval2->getVerticalKeyPoints().size(), 0);
    }else{
        EXPECT_TRUE(false);
    }
    if (interval3 != nullptr) {
        EXPECT_GT(interval3->getHorizontalKeyPoints().size(), 0);
    }else{
        EXPECT_TRUE(false);
    }
    if (interval4 != nullptr) {
        EXPECT_GT(interval4->getHorizontalKeyPoints().size(), 0);
    }else{
        EXPECT_TRUE(false);
    }
}

// 测试邻居关系构建
TEST_F(PreprocessTest, NeighborRelations) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    const auto* interval1 = preprocess.findKeyInterval({4,0,2});
    const auto* interval2 = preprocess.findKeyInterval({4,5,7});
    
    // 只测试存在的interval
    if (interval1 != nullptr) {
        EXPECT_GT(interval1->neighbors.size(), 0);
    }else{
        EXPECT_TRUE(false);
    }
    if (interval2 != nullptr) {
        EXPECT_GT(interval2->neighbors.size(), 0);
    }else{
        EXPECT_TRUE(false);
    }
}

// 测试transition vertices
TEST_F(PreprocessTest, TransitionVertices) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    const auto* interval1 = preprocess.findKeyInterval({2,0,7});
    const auto* interval2 = preprocess.findKeyInterval({5,0,7});
    
    EXPECT_NE(interval1, nullptr);
    EXPECT_NE(interval2, nullptr);
    
    Preprocess::IntervalKey key1{4,0,2};
    Preprocess::IntervalKey key2{4,5,7};
    
    // 使用find()方法安全地检查transition vertices是否存在
    auto it1 = interval1->transitionVertices.find(key1);
    if (it1 != interval1->transitionVertices.end()) {
        auto it1_2 = it1->second.find(key2);
        if (it1_2 != it1->second.end()) {
            EXPECT_EQ(it1_2->second, Vertex(4,2));
            logger::log_info("true1");
        }
    }else{
        EXPECT_TRUE(false);
    }
    
    auto it1_reverse = interval1->transitionVertices.find(key2);
    if (it1_reverse != interval1->transitionVertices.end()) {
        auto it1_reverse_1 = it1_reverse->second.find(key1);
        if (it1_reverse_1 != it1_reverse->second.end()) {
            EXPECT_EQ(it1_reverse_1->second, Vertex(4,2));
            logger::log_info("true2");
        }
    }else{
        EXPECT_TRUE(false);
    }
    
    auto it2 = interval2->transitionVertices.find(key1);
    if (it2 != interval2->transitionVertices.end()) {
        auto it2_2 = it2->second.find(key2);
        if (it2_2 != it2->second.end()) {
            EXPECT_EQ(it2_2->second, Vertex(4,5));
            logger::log_info("true3");
        }
    }else{
        EXPECT_TRUE(false);
    }
    
    auto it2_reverse = interval2->transitionVertices.find(key2);
    if (it2_reverse != interval2->transitionVertices.end()) {
        auto it2_reverse_1 = it2_reverse->second.find(key1);
        if (it2_reverse_1 != it2_reverse->second.end()) {
            EXPECT_EQ(it2_reverse_1->second, Vertex(4,5));
            logger::log_info("true4");
        }
    }else{
        EXPECT_TRUE(false);
    }
}



// 测试边界情况
TEST_F(PreprocessTest, EdgeCases) {
    // 测试1x1地图
    std::vector<std::vector<int>> tinyMap = {{0}};
    Preprocess tinyPreprocess(tinyMap);
    tinyPreprocess.preprocess();
    
    EXPECT_EQ(tinyPreprocess.getHeight(), 1);
    EXPECT_EQ(tinyPreprocess.getWidth(), 1);
    
    // 测试全障碍物地图
    std::vector<std::vector<int>> allObstaclesMap = {
        {1, 1, 1},
        {1, 1, 1},
        {1, 1, 1}
    };
    Preprocess obstaclesPreprocess(allObstaclesMap);
    obstaclesPreprocess.preprocess();

    const auto& verticalIntervals = obstaclesPreprocess.getVerticalIntervals();
    // 全障碍物地图可能没有vertical intervals，或者有但都是空的
    for (const auto& [y, rowIntervals] : verticalIntervals) {
        EXPECT_EQ(rowIntervals.size(), 0);
    }
    
    const auto& keyIntervals = obstaclesPreprocess.getKeyIntervals();
    // 全障碍物地图应该没有key intervals
    EXPECT_EQ(keyIntervals.size(), 0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 