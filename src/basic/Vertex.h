#ifndef VERTEX_H
#define VERTEX_H

#include <functional>

struct Vertex {
    // x is row coordinate, y is column coordinate
    int x, y;

    explicit Vertex(int x = 0, int y = 0) : x(x), y(y) {}

    bool operator==(const Vertex& other) const {
        return x == other.x && y == other.y;
    }

    Vertex operator+(const Vertex& other) const {
        return Vertex(x + other.x, y + other.y);
    }

    Vertex operator-(const Vertex& other) const {
        return Vertex(x - other.x, y - other.y);
    }

    bool operator!=(const Vertex& other) const {
        return !(*this == other);
    }
};

// provide specialization for std::hash
namespace std {
    template<>
    struct hash<Vertex> {
        size_t operator()(const Vertex& v) const {
            return hash<int>()(v.x) ^ (hash<int>()(v.y) << 1);
        }
    };
}

#endif //VERTEX_H
