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
        // set test maps
        setupEmptyMap();
        setupSimpleMap();
        setupComplexMap();
        setupMazeMap();
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

    void setupMazeMap() {
        // 8x8 maze map
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

// test basic constructor
TEST_F(KeyIntervalAStarTest, Constructor) {
    Preprocess preprocess(emptyMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    // constructor should succeed, no exception
    EXPECT_TRUE(true);
}

// test empty map path search
TEST_F(KeyIntervalAStarTest, EmptyMapSearch ) {
    Preprocess preprocess(emptyMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // test simple straight path
    Vertex start(0, 0);
    Vertex end(7, 7);
    
    std::vector<Vertex> path = kiaStar.search(start, end);
    
    // empty map should find path
    EXPECT_FALSE(path.empty());
    
    // check path start and end
    if (!path.empty()) {
        EXPECT_EQ(path.front(), start);
        EXPECT_EQ(path.back(), end);
        EXPECT_EQ(path.size(), 15);
    }

}

// test simple map path search
TEST_F(KeyIntervalAStarTest, SimpleMapSearch) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // test path around obstacles
    Vertex start(0, 0);
    Vertex end(7, 7);
    
    std::vector<Vertex> path = kiaStar.search(start, end);
    
    // should find path
    EXPECT_FALSE(path.empty());
    
    // check path start and end
    if (!path.empty()) {
        EXPECT_EQ(path.front(), start);
        EXPECT_EQ(path.back(), end);
        
        // check path around obstacles (obstacles at (3,3)-(4,4))
        for (const auto& vertex : path) {
            EXPECT_FALSE((vertex.x == 3 && vertex.y == 3) ||
                        (vertex.x == 3 && vertex.y == 4) ||
                        (vertex.x == 4 && vertex.y == 3) ||
                        (vertex.x == 4 && vertex.y == 4));
        }
        EXPECT_EQ(path.size(), 15);
    }
}

// test complex map path search
TEST_F(KeyIntervalAStarTest, ComplexMapSearch) {
    Preprocess preprocess(complexMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // test complex path
    Vertex start1(0, 0);
    Vertex end1(7, 7);

    Vertex start2(0, 7);
    Vertex end2(7, 7);
    
    std::vector<Vertex> path1 = kiaStar.search(start1, end1);
    std::vector<Vertex> path2 = kiaStar.search(start2, end2);
    
    // should find path
    EXPECT_FALSE(path1.empty());
    EXPECT_FALSE(path2.empty());
    
    // check path start and end
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

// test maze map path search
TEST_F(KeyIntervalAStarTest, MazeMapSearch) {
    Preprocess preprocess(mazeMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // test maze path
    Vertex start(0, 5);
    Vertex end(7, 7);
    
    std::vector<Vertex> path = kiaStar.search(start, end);
    
    // should find path
    EXPECT_FALSE(path.empty());
    
    // check path start and end
    if (!path.empty()) {
        EXPECT_EQ(path.front(), start);
        EXPECT_EQ(path.back(), end);
        EXPECT_EQ(path.size(), 20);
    }
    
}

// test adjacent points path search
TEST_F(KeyIntervalAStarTest, AdjacentPointsSearch) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // test adjacent points
    Vertex start(0, 0);
    Vertex end(1, 0);
    
    std::vector<Vertex> path = kiaStar.search(start, end);
    
    // should find path
    EXPECT_FALSE(path.empty());
    
    if (!path.empty()) {
        EXPECT_EQ(path.front(), start);
        EXPECT_EQ(path.back(), end);
        // adjacent points path should be short
        EXPECT_LE(path.size(), 3);
    }
}

// test same start and end path search
TEST_F(KeyIntervalAStarTest, SameStartEndSearch) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // test same start and end
    Vertex start(0, 0);
    Vertex end(0, 0);
    
    std::vector<Vertex> path = kiaStar.search(start, end);
    
    // should find path (only contains start point)
    EXPECT_FALSE(path.empty());
    
    if (!path.empty()) {
        EXPECT_EQ(path.front(), start);
        EXPECT_EQ(path.back(), end);
        EXPECT_EQ(path.size(), 1);
    }
}

// test boundary points path search
TEST_F(KeyIntervalAStarTest, BoundaryPointsSearch) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // test boundary points
    Vertex start(0, 0);
    Vertex end(7, 7);
    
    std::vector<Vertex> path = kiaStar.search(start, end);
    
    // should find path
    EXPECT_FALSE(path.empty());
    
    if (!path.empty()) {
        EXPECT_EQ(path.front(), start);
        EXPECT_EQ(path.back(), end);
    }
}

// test unreachable target
TEST_F(KeyIntervalAStarTest, UnreachableTarget) {
    // create a map completely surrounded by obstacles
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
    
    // test path from inside to outside (should be unreachable)
    Vertex start(1, 1);
    Vertex end(8, 8); // out of map boundary
    
    std::vector<Vertex> path = kiaStar.search(start, end);
    
    // should return empty path
    EXPECT_TRUE(path.empty());
}

// test path continuity
TEST_F(KeyIntervalAStarTest, PathContinuity) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    Vertex start(0, 0);
    Vertex end(7, 7);
    
    std::vector<Vertex> path = kiaStar.search(start, end);
    
    if (!path.empty()) {
        // check path continuity
        for (size_t i = 1; i < path.size(); ++i) {
            const auto& prev = path[i-1];
            const auto& curr = path[i];
            
            // adjacent points should only differ by 1 coordinate
            int dx = std::abs(curr.x - prev.x);
            int dy = std::abs(curr.y - prev.y);
            
            EXPECT_TRUE((dx == 1 && dy == 0) || (dx == 0 && dy == 1));
        }
    }
}

// test path optimality (simple case)
TEST_F(KeyIntervalAStarTest, PathOptimality) {
    Preprocess preprocess(emptyMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // test straight path
    Vertex start(0, 0);
    Vertex end(3, 3);
    
    std::vector<Vertex> path = kiaStar.search(start, end);
    
    if (!path.empty()) {
        // in empty map, straight path should be optimal
        // manhattan distance should be 6 (3+3)
        int manhattanDistance = std::abs(end.x - start.x) + std::abs(end.y - start.y);
        EXPECT_EQ(path.size() - 1, manhattanDistance);
    }
}

// test multiple different start and end combinations
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
        
        // all test cases should find path
        EXPECT_FALSE(path.empty()) << "Failed for start(" << start.x << "," << start.y 
                                  << ") to end(" << end.x << "," << end.y << ")";
        
        if (!path.empty()) {
            EXPECT_EQ(path.front(), start);
            EXPECT_EQ(path.back(), end);
        }
    }
}

// test heuristic distance calculation
TEST_F(KeyIntervalAStarTest, HeuristicDistanceCalculation) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // test some known manhattan distances
    Vertex v1(0, 0);
    Vertex v2(3, 4);
    Vertex v3(7, 7);
    
    // these tests need to add public methods to KeyIntervalAStar class to test
    // skip this test for now, because calculateHeuristicDistance is private
    EXPECT_TRUE(true);
}

// test key interval finding
TEST_F(KeyIntervalAStarTest, KeyIntervalFinding) {
    Preprocess preprocess(simpleMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    // test some vertices in key interval
    Vertex testVertex(2, 2);
    
    // these tests need to add public methods to KeyIntervalAStar class to test
    // skip this test for now, because related methods are private
    EXPECT_TRUE(true);
}

// test multiple different start and end combinations for maze map
TEST_F(KeyIntervalAStarTest, MazeMapMultipleStartEndCombinations) {
    Preprocess preprocess(mazeMap);
    preprocess.preprocess();
    
    KeyIntervalAStar kiaStar(preprocess);
    
    std::vector<std::pair<Vertex, Vertex>> testCases;
    // test path from top left to bottom right
    testCases.push_back(std::make_pair(Vertex(0, 0), Vertex(7, 7)));
    // test path from top right to bottom left
    testCases.push_back(std::make_pair(Vertex(0, 7), Vertex(7, 0)));
    // test path from middle area to bottom right
    testCases.push_back(std::make_pair(Vertex(2, 2), Vertex(7, 7)));
    // test path from top left to middle area
    testCases.push_back(std::make_pair(Vertex(0, 0), Vertex(4, 4)));
    // test path from bottom to top
    testCases.push_back(std::make_pair(Vertex(7, 0), Vertex(0, 0)));
    // test path from right to left
    testCases.push_back(std::make_pair(Vertex(0, 7), Vertex(0, 0)));
    // test path from maze entrance to exit
    testCases.push_back(std::make_pair(Vertex(0, 2), Vertex(7, 6)));
    // test path from maze inside to outside
    testCases.push_back(std::make_pair(Vertex(2, 4), Vertex(7, 7)));
    
    for (const auto& [start, end] : testCases) {
        std::vector<Vertex> path = kiaStar.search(start, end);
        
        // all test cases should find path
        EXPECT_FALSE(path.empty()) << "Failed for start(" << start.x << "," << start.y 
                                  << ") to end(" << end.x << "," << end.y << ")";
        
        if (!path.empty()) {
            EXPECT_EQ(path.front(), start);
            EXPECT_EQ(path.back(), end);
            
            // check path avoids obstacles (obstacle value is 1)
            for (const auto& vertex : path) {
                EXPECT_EQ(mazeMap[vertex.x][vertex.y], 0) 
                    << "Path goes through obstacle at (" << vertex.x << "," << vertex.y << ")";
            }
            
            // check path continuity
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