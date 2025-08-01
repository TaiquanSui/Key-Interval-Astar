#include <gtest/gtest.h>
#include "../src/KIAstar/Preprocess.h"
#include <vector>
#include <set>
#include "../src/utilities/Log.h"

class PreprocessTest : public ::testing::Test {
protected:
    void SetUp() override {
        // set test maps
        setupEmptyMap();
        setupSimpleMap();
        setupComplexMap();
    }

    void setupEmptyMap() {
        // 8x8 empty map
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
        // 8x8 simple map, with an obstacle in the middle
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
        // 8x8 complex map, with multiple obstacles
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

// test basic constructor and map size
TEST_F(PreprocessTest, ConstructorAndMapSize) {
    Preprocess preprocess(emptyMap);
    
    EXPECT_EQ(preprocess.getHeight(), 8);
    EXPECT_EQ(preprocess.getWidth(), 8);
    
    const auto& grid = preprocess.getGrid();
    EXPECT_EQ(grid.size(), 8);
    EXPECT_EQ(grid[0].size(), 8);
}

// test empty map preprocessing
TEST_F(PreprocessTest, EmptyMapPreprocess) {
    Preprocess preprocess(emptyMap);
    preprocess.preprocess();
    
    // empty map should have no key intervals
    const auto& verticalIntervals = preprocess.getVerticalIntervals();
    const auto& keyIntervals = preprocess.getKeyIntervals();
    EXPECT_EQ(verticalIntervals.size(), 8);
    EXPECT_EQ(keyIntervals.size(), 0);
}

// test simple map preprocessing
TEST_F(PreprocessTest, SimpleMapPreprocess) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    const auto& keyIntervals = preprocess.getKeyIntervals();
    // simple map should have some key intervals (at obstacle boundaries)
    EXPECT_EQ(keyIntervals.size(), 4);
}

// test findKeyInterval method
TEST_F(PreprocessTest, FindKeyInterval) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    const auto& keyIntervals = preprocess.getKeyIntervals();
    
    // test find existing key interval
    for (const auto& [key, interval] : keyIntervals) {
        auto found = preprocess.findKeyInterval(key);
        EXPECT_TRUE(found.has_value());
        EXPECT_EQ(found->getY(), interval.getY());
        EXPECT_EQ(found->getStart(), interval.getStart());
        EXPECT_EQ(found->getEnd(), interval.getEnd());
    }
    
    // test find non-existing key interval
    Preprocess::IntervalKey nonExistentKey{999, 999, 999};
    auto notFound = preprocess.findKeyInterval(nonExistentKey);
    EXPECT_FALSE(notFound.has_value());
}


// test key point extraction
TEST_F(PreprocessTest, KeyPointExtraction) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    auto interval1 = preprocess.findKeyInterval(Preprocess::IntervalKey{4,0,2});
    auto interval2 = preprocess.findKeyInterval(Preprocess::IntervalKey{4,5,7});
    auto interval3 = preprocess.findKeyInterval(Preprocess::IntervalKey{2,0,7});
    auto interval4 = preprocess.findKeyInterval(Preprocess::IntervalKey{5,0,7});
    
    // only test existing interval
    if (interval1.has_value()) {
        EXPECT_TRUE(interval1->getUpVertex().has_value() || interval1->getDownVertex().has_value());
    }else{
        EXPECT_TRUE(false);
    }
    if (interval2.has_value()) {
        EXPECT_TRUE(interval2->getUpVertex().has_value() || interval2->getDownVertex().has_value());
    }else{
        EXPECT_TRUE(false);
    }
    if (interval3.has_value()) {
        EXPECT_GT(interval3->getHorizontalKeyPoints().size(), 0);
    }else{
        EXPECT_TRUE(false);
    }
    if (interval4.has_value()) {
        EXPECT_GT(interval4->getHorizontalKeyPoints().size(), 0);
    }else{
        EXPECT_TRUE(false);
    }
}

// test neighbor relations building
TEST_F(PreprocessTest, NeighborRelations) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    auto interval1 = preprocess.findKeyInterval(Preprocess::IntervalKey{4,0,2});
    auto interval2 = preprocess.findKeyInterval(Preprocess::IntervalKey{4,5,7});
    
    // only test existing interval
    if (interval1.has_value()) {
        EXPECT_GT(interval1->neighbors.size(), 0);
    }else{
        EXPECT_TRUE(false);
    }
    if (interval2.has_value()) {
        EXPECT_GT(interval2->neighbors.size(), 0);
    }else{
        EXPECT_TRUE(false);
    }
}

// test transition vertices
TEST_F(PreprocessTest, TransitionVertices) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    auto interval1 = preprocess.findKeyInterval(Preprocess::IntervalKey{2,0,7});
    auto interval2 = preprocess.findKeyInterval(Preprocess::IntervalKey{5,0,7});
    
    EXPECT_TRUE(interval1.has_value());
    EXPECT_TRUE(interval2.has_value());
    
    Preprocess::IntervalKey key1{4,0,2};
    Preprocess::IntervalKey key2{4,5,7};
    
    // use find() method to safely check if transition vertices exist
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



// test edge cases
TEST_F(PreprocessTest, EdgeCases) {
    // test 1x1 map
    std::vector<std::vector<int>> tinyMap = {{0}};
    Preprocess tinyPreprocess(tinyMap);
    tinyPreprocess.preprocess();
    
    EXPECT_EQ(tinyPreprocess.getHeight(), 1);
    EXPECT_EQ(tinyPreprocess.getWidth(), 1);
    
    // test all obstacles map
    std::vector<std::vector<int>> allObstaclesMap = {
        {1, 1, 1},
        {1, 1, 1},
        {1, 1, 1}
    };
    Preprocess obstaclesPreprocess(allObstaclesMap);
    obstaclesPreprocess.preprocess();

    const auto& verticalIntervals = obstaclesPreprocess.getVerticalIntervals();
    // all obstacles map may have no vertical intervals, or have but are empty
    for (const auto& [y, rowIntervals] : verticalIntervals) {
        EXPECT_EQ(rowIntervals.size(), 0);
    }
    
    const auto& keyIntervals = obstaclesPreprocess.getKeyIntervals();
    // all obstacles map should have no key intervals
    EXPECT_EQ(keyIntervals.size(), 0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 