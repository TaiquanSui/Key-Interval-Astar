#pragma once

#include <vector>
#include <set>
#include <map>
#include <unordered_map>
#include <iostream>
#include <optional>
#include "../basic/Vertex.h"
#include "../utilities/GridUtility.h"


class Preprocess {
public:
    // define trend enum
    enum class Trend {
        UNCHANGED = 0,
        DECREASING = 1,
        INCREASING = 2
    };

    const char* trendToString(Trend t) {
        switch (t) {
            case Trend::UNCHANGED:   return "UNCHANGED";
            case Trend::DECREASING:  return "DECREASING";
            case Trend::INCREASING:  return "INCREASING";
            default: return "UNKNOWN";
        }
    }

    // define key point direction
    enum class Direction {
        UP = 0,     // can pass from above
        DOWN = 1,   // can pass from below
        LEFT = 2,   // can pass from left
        RIGHT = 3   // can pass from right
    };

    // define key point structure
    struct KeyPoint {
        Vertex point;
        Direction direction;

        KeyPoint(const Vertex& p, Direction d) : point(p), direction(d) {}
        
        bool operator<(const KeyPoint& other) const {
            if (point.x != other.point.x) {
                return point.x < other.point.x;
            }
            if (point.y != other.point.y) {
                return point.y < other.point.y;
            }
            return direction < other.direction;
        }
    };

    // define interval structure for scan
    struct Interval {
        int start;
        int end;
        Trend start_mark;  // start trend mark
        Trend end_mark;    // end trend mark

        Interval(int s, int e) : 
            start(s), end(e), 
            start_mark(Trend::UNCHANGED), 
            end_mark(Trend::UNCHANGED) {}
    };

    // define key structure for hashmap
    struct IntervalKey {
        int y;
        int start;
        int end;
        
        bool operator==(const IntervalKey& other) const {
            return y == other.y && start == other.start && end == other.end;
        }

        bool operator<(const IntervalKey& other) const {
            if (y != other.y) return y < other.y;
            if (start != other.start) return start < other.start;
            return end < other.end;
        }
    };

    // define hash function
    struct IntervalKeyHash {
        size_t operator()(const IntervalKey& key) const {
            return std::hash<int>()(key.y) ^ 
                   (std::hash<int>()(key.start) << 1) ^ 
                   (std::hash<int>()(key.end) << 2);
        }
    };

    // define vertical interval structure
    struct VerticalInterval {
        int y;              // y coordinate
        int start;          // x coordinate start
        int end;            // x coordinate end
        std::optional<Vertex> upVertex;             // vertical direction up key point
        std::optional<Vertex> downVertex;           // vertical direction down key point
        std::vector<KeyPoint> horizontalKeyPoints;  // horizontal direction key points (LEFT/RIGHT)
        std::vector<IntervalKey> leftNeighbors;     // left neighbors
        std::vector<IntervalKey> rightNeighbors;    // right neighbors
        std::optional<std::pair<IntervalKey, IntervalKey>> leftKeyInterval;   // (key interval, key interval direct neighbor)
        std::optional<std::pair<IntervalKey, IntervalKey>> rightKeyInterval;  // (key interval, key interval direct neighbor)
        
        // constructor with parameters
        VerticalInterval(int y_pos, int s, int e) : 
            y(y_pos), start(s), end(e) {}

        VerticalInterval() : y(0), start(0), end(0) {}
    };

    // define neighbor triple structure
    struct NeighborTriple {
        IntervalKey currentDirectNeighbor;    // current interval direct neighbor
        IntervalKey neighborDirectNeighbor;       // neighbor key interval direct neighbor
        IntervalKey neighborKeyInterval;      // neighbor key interval
        
        NeighborTriple(const IntervalKey& current, const IntervalKey& path, const IntervalKey& neighbor)
            : currentDirectNeighbor(current), neighborDirectNeighbor(path), neighborKeyInterval(neighbor) {}
        
        bool operator==(const NeighborTriple& other) const {
            return currentDirectNeighbor == other.currentDirectNeighbor &&
                   neighborDirectNeighbor == other.neighborDirectNeighbor &&
                   neighborKeyInterval == other.neighborKeyInterval;
        }
        
        bool operator!=(const NeighborTriple& other) const {
            return !(*this == other);
        }
    };

    // define key interval structure
    struct KeyInterval {
        const VerticalInterval* verticalInterval;  // pointer to corresponding VerticalInterval
        // directly store neighbor triple set
        std::vector<NeighborTriple> neighbors;
        // store transition vertices between direct neighbors
        std::unordered_map<IntervalKey, std::unordered_map<IntervalKey, Vertex, IntervalKeyHash>, IntervalKeyHash> transitionVertices;

        KeyInterval(const VerticalInterval* vInterval) : verticalInterval(vInterval) {}
        KeyInterval() = default;
        
        // convenient access method
        int getY() const { return verticalInterval->y; }
        int getStart() const { return verticalInterval->start; }
        int getEnd() const { return verticalInterval->end; }
        const std::vector<KeyPoint>& getHorizontalKeyPoints() const { return verticalInterval->horizontalKeyPoints; }
        std::optional<Vertex> getUpVertex() const { return verticalInterval->upVertex; }
        std::optional<Vertex> getDownVertex() const { return verticalInterval->downVertex; }
    };

    // constructor
    Preprocess(const std::vector<std::vector<int>>& grid);

    // execute preprocessing
    void preprocess();
    double getPreprocessTime() const { return preprocess_time_; }

    // get map size
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }

    // get key intervals
    const std::unordered_map<IntervalKey, KeyInterval, IntervalKeyHash>& getKeyIntervals() const { 
        return keyIntervals_; 
    }

    // get vertical intervals
    const std::unordered_map<int, std::unordered_map<int, VerticalInterval>>& getVerticalIntervals() const { 
        return verticalIntervals_; 
    }

    // find KeyInterval by key
    const std::optional<KeyInterval> findKeyInterval(const IntervalKey& key) const {
        auto it = keyIntervals_.find(key);
        return it != keyIntervals_.end() ? std::optional<KeyInterval>(it->second) : std::nullopt;
    }

    const std::optional<KeyInterval> findKeyInterval(const VerticalInterval& interval) const {
        for(const auto& [_, keyInterval] : keyIntervals_) {
            if(keyInterval.verticalInterval == &interval) {
                return std::optional<KeyInterval>(keyInterval);
            }
        }
        return std::nullopt;
    }

    // find vertical interval containing vertex
    std::optional<VerticalInterval> findContainingVerticalInterval(const Vertex& vertex) const;
    // check if vertex is in some key interval
    static bool isVertexInKeyInterval(const Vertex& vertex, const IntervalKey& key);
    // find transition vertex
    std::optional<Vertex> findTransitionVertex(const IntervalKey& keyInterval,
                                              const IntervalKey& directNeighbor1,
                                              const IntervalKey& directNeighbor2) const;

    // general scan method
    void processScan(bool isVertical);

    // process key points
    // void processKeyPoint(std::vector<Interval>& currentIntervals, 
    //                     std::vector<Interval>& nextIntervals, bool isVertical, int fixed);

    // process neighbor relations
    void processNeighborPair(const IntervalKey& key,
                           const std::vector<IntervalKey>& directNeighbors,
                           Direction dir);

    // process transition vertices
    void processTransitionVertices(const IntervalKey& key,
                            const VerticalInterval& currentInterval,
                            const std::vector<IntervalKey>& directNeighbors,
                                    Direction dir);

    // get map data
    const std::vector<std::vector<int>>& getGrid() const { return grid_; }
    
    // get memory usage of preprocessing (bytes)
    size_t getMemoryUsage() const;

private:
    // map data
    const std::vector<std::vector<int>>& grid_;
    // map size
    int width_;
    int height_;
    
    // store all vertical intervals
    std::unordered_map<int, std::unordered_map<int, VerticalInterval>> verticalIntervals_;
    
    // store all key intervals (using hashmap)
    std::unordered_map<IntervalKey, KeyInterval, IntervalKeyHash> keyIntervals_;

    double preprocess_time_ = 0.0;

    // get maximum movable space
    void getMaxMovableSpace(int fixed, int range, bool isVertical, std::vector<Interval>& intervals) const;

    // build key interval
    void buildKeyIntervals();

    // build neighbor relations between key intervals
    void buildNeighbors();

    // update interval trends
    void processCurrentIntervals(std::vector<Interval>& prevIntervals, std::vector<Interval>& currentIntervals, 
                            std::vector<Interval>& nextIntervals, bool isVertical, int fixed);
    
}; 


// add operator<< for Preprocess class
inline std::ostream& operator<<(std::ostream& os, const Preprocess::IntervalKey& key) {
    os << "[" << key.y << "," << key.start << "," << key.end << "]";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const Preprocess::Direction& direction) {
    os << "[" << direction << "]";
    return os;
}   

inline std::ostream& operator<<(std::ostream& os, const Preprocess::KeyPoint& keyPoint) {
    os << "[" << keyPoint.point.x << "," << keyPoint.point.y << "," << keyPoint.direction << "]";
    return os;
}   

inline std::ostream& operator<<(std::ostream& os, const Preprocess::Interval& interval) {
    os << "[" << interval.start << "," << interval.end << "]";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const Preprocess::VerticalInterval& verticalInterval) {
    os << "[" << verticalInterval.y << "," << verticalInterval.start << "," << verticalInterval.end << "]";
    return os;
}
// add operator<< for std::pair<IntervalKey, IntervalKey>
inline std::ostream& operator<<(std::ostream& os, const std::pair<Preprocess::IntervalKey, Preprocess::IntervalKey>& pair) {
    os << "(" << pair.first << "->" << pair.second << ")";
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const Preprocess::NeighborTriple& triple) {
    os << "(" << triple.currentDirectNeighbor << "->" << triple.neighborDirectNeighbor << "->" << triple.neighborKeyInterval << ")";
    return os;
}

