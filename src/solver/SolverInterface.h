#ifndef SOLVER_INTERFACE_H
#define SOLVER_INTERFACE_H

#include "../basic/Vertex.h"
#include <vector>
#include <string>

// general solver interface
class SolverInterface {
public:
    virtual ~SolverInterface() = default;
    virtual std::vector<Vertex> search(const Vertex& start, const Vertex& target) = 0;
    virtual std::string get_name() const = 0;
    virtual int getExpandedNodes() const = 0;
    virtual void resetExpandedNodes() = 0;
    virtual double getSearchTime() const = 0;
    virtual void resetSearchTime() = 0;
    
    // memory monitoring interface
    virtual size_t getPreprocessMemoryUsage() const = 0;  // get memory usage of preprocessing (bytes)
    virtual size_t getSearchMemoryIncrease() const = 0;   // get memory usage of search (bytes)
    virtual void resetSearchMemoryUsage() = 0;            // reset search memory usage
    // new: get preprocessing time (seconds)
    virtual double getPreprocessTime() const = 0;
    
    // new: preprocessing interface
    virtual void preprocess(const std::vector<std::vector<int>>& grid) = 0;
};

#endif // SOLVER_INTERFACE_H 