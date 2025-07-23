#ifndef SOLVER_INTERFACE_H
#define SOLVER_INTERFACE_H

#include "../basic/Vertex.h"
#include <vector>
#include <string>

// 通用的solver接口
class SolverInterface {
public:
    virtual ~SolverInterface() = default;
    virtual std::vector<Vertex> search(const Vertex& start, const Vertex& target) = 0;
    virtual std::string get_name() const = 0;
    virtual int getExpandedNodes() const = 0;
    virtual void resetExpandedNodes() = 0;
    virtual double getSearchTime() const = 0;
    virtual void resetSearchTime() = 0;
    
    // 内存监控接口
    virtual size_t getPreprocessMemoryUsage() const = 0;  // 获取预处理占用的内存（字节）
    virtual size_t getSearchMemoryIncrease() const = 0;   // 获取搜索过程中增加的内存（字节）
    virtual void resetSearchMemoryUsage() = 0;            // 重置搜索内存统计
    // 新增：获取预处理时间（秒）
    virtual double getPreprocessTime() const = 0;
    
    // 新增：预处理接口
    virtual void preprocess(const std::vector<std::vector<int>>& grid) = 0;
};

#endif // SOLVER_INTERFACE_H 