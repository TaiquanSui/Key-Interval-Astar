#ifndef HEURISTIC_H
#define HEURISTIC_H

#include "../basic/Vertex.h"
#include "../utilities/GridUtility.h"
#include <cmath>


inline double heuristic(const Vertex& a, const Vertex& b) {
    return utils::manhattanDistance(a, b);
}

#endif // HEURISTIC_H
