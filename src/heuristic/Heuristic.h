#ifndef HEURISTIC_H
#define HEURISTIC_H

#include "../basic/Vertex.h"
#include "../utilities/GridUtility.h"
#include <cmath>

inline double heuristic_octile(const Vertex& a, const Vertex& b) {
    return utils::octileDistance(a, b);
}

inline double heuristic_manhattan(const Vertex& a, const Vertex& b) {
    return utils::manhattanDistance(a, b);
}

inline double heuristic_euclidean(const Vertex& a, const Vertex& b) {
    return utils::euclideanDistance(a, b);
}

#endif // HEURISTIC_H
