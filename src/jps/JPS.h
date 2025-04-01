#ifndef JPS_H
#define JPS_H

#include "../basic/Vertex.h"
#include "../astar/AStar.h"
#include <vector>


std::vector<Vertex> jump_point_search(const Vertex& start, 
                         const Vertex& goal, 
                         const std::vector<std::vector<int>>& grid);

#endif // JPS_H 