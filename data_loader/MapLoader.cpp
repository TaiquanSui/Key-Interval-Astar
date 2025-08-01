#include "MapLoader.h"
#include "../src/utilities/Log.h"
#include <fstream>
#include <sstream>

std::vector<std::vector<int>> load_map(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("cannot open map file: " + filename);
    }

    std::string line;
    int width = 0, height = 0;

    // read header information
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        std::string key;
        iss >> key;
        
        if (key == "height") {
            iss >> height;
        } else if (key == "width") {
            iss >> width;
        } else if (key == "map") {
            break;  // map content starts
        }
    }

    if (width <= 0 || height <= 0) {
        throw std::runtime_error("invalid map size");
    }

    // no longer swap width and height
    // std::swap(width, height);

    // read map content - create map in normal order
    std::vector<std::vector<int>> map(height, std::vector<int>(width, 0));
    int row = 0;
    int col = 0;
    char c;
    
    // read map content character by character
    while (file.get(c) && row < height) {
        // skip newline and carriage return
        if (c == '\n' || c == '\r') {
            continue;
        }
        
        // process current character
        switch(c) {
            case '.':
            case 'G':
                map[row][col] = 0;  // passable
                break;
            case '@':
            case 'O':
            case 'T':
                map[row][col] = 1;  // impassable
                break;
            // case 'S':
            // case 'W':
            //     map[row][col] = 0;  // temporarily视为可通行
            //     break;
            default:
                throw std::runtime_error("map contains unknown character: " + std::string(1, c));
        }
        
        // update position
        ++col;
        if (col >= width) {
            col = 0;
            ++row;
        }
    }

    if (row != height) {
        throw std::runtime_error("map data is insufficient");
    }

    return map;
}

std::vector<Agent> load_scen(const std::string& filename, const std::vector<std::vector<int>>& grid) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("cannot open scenario file: " + filename);
    }

    std::vector<Agent> agents;
    std::string line;
    int agent_id = 0;  // assign increasing ID to each agent

    // skip version line
    std::getline(file, line);

    // read each scenario
    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::istringstream iss(line);
        std::string bucket, map_name;
        int width, height, start_x, start_y, goal_x, goal_y;
        double optimal_length;

        // parse scenario line
        if (!(iss >> bucket >> map_name >> width >> height 
              >> start_x >> start_y >> goal_x >> goal_y >> optimal_length)) {
            continue;  // skip invalid line
        }

        agents.emplace_back(
            agent_id++,
            Vertex(start_y, start_x),
            Vertex(goal_y, goal_x)
        );
    }

    return agents;
}
