#ifndef HPASTAR_H
#define HPASTAR_H

#include "../basic/Vertex.h"
#include "../solver/SolverInterface.h"
#include "../utilities/MemoryUtility.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <queue>
#include <string>

// 抽象节点结构 - 移除cluster_id
struct AbstractNode {
    Vertex exit;              // 出口点
    double g_cost;            // 从起点到该节点的代价
    double h_cost;            // 启发式代价
    std::shared_ptr<AbstractNode> parent;
    
    AbstractNode(Vertex ext, double g = 0, double h = 0, 
                 std::shared_ptr<AbstractNode> p = nullptr)
        : exit(ext), g_cost(g), h_cost(h), parent(p) {}
    
    double f() const { return g_cost + h_cost; }
};

// 抽象节点比较器
struct AbstractNodeComparator {
    bool operator()(const std::shared_ptr<AbstractNode>& a, 
                   const std::shared_ptr<AbstractNode>& b) const {
        if (std::abs(a->f() - b->f()) > 1e-6) {
            return a->f() > b->f();
        }
        return a->g_cost < b->g_cost;
    }
};

// 聚类结构
struct Cluster {
    int id;
    Vertex top_left;          // 左上角坐标
    Vertex bottom_right;      // 右下角坐标
    std::unordered_set<Vertex> exits;  // 出口点列表（简化）
    
    Cluster(int cluster_id, Vertex tl, Vertex br) 
        : id(cluster_id), top_left(tl), bottom_right(br), exits() {}
    
    bool contains(const Vertex& v) const {
        return v.x >= top_left.x && v.x <= bottom_right.x &&
               v.y >= top_left.y && v.y <= bottom_right.y;
    }
    
    int width() const { return bottom_right.x - top_left.x + 1; }
    int height() const { return bottom_right.y - top_left.y + 1; }
};

// 抽象边结构 - 只保留cost和path
struct AbstractEdge {
    double cost;
    AbstractEdge(double c) : cost(c) {}
    AbstractEdge() : cost(0.0) {}
};


// HPA*算法类
class HPAStar : public SolverInterface {
private:
    std::vector<std::vector<int>> grid_;
    std::vector<Cluster> clusters_;
    // 新版嵌套map
    std::unordered_map<Vertex, std::unordered_map<Vertex, AbstractEdge>> abstract_edges_;
    int cluster_size_;
    int expanded_nodes_;
    double search_time_;
    double preprocess_time_;
    std::string name_;
    size_t memory_before_;
    size_t memory_after_;
    size_t preprocess_memory_;

public:
    HPAStar(const std::string& name = "HPAStar", int cluster_size = 10) 
        : cluster_size_(cluster_size), expanded_nodes_(0), search_time_(0.0), 
          preprocess_time_(0.0), name_(name), memory_before_(0), memory_after_(0), 
          preprocess_memory_(0) {}
    
    // 实现SolverInterface接口
    std::vector<Vertex> search(const Vertex& start, const Vertex& target) override;
    std::string get_name() const override { return name_; }
    int getExpandedNodes() const override { return expanded_nodes_; }
    void resetExpandedNodes() override { expanded_nodes_ = 0; }
    double getSearchTime() const override { return search_time_; }
    void resetSearchTime() override { search_time_ = 0.0; }
    
    // 内存监控接口实现
    size_t getPreprocessMemoryUsage() const override { return preprocess_memory_; }
    size_t getSearchMemoryIncrease() const override;
    void resetSearchMemoryUsage() override;
    double getPreprocessTime() const override { return preprocess_time_; }
    
    // 新增：直接计算内存使用量
    size_t getMemoryUsage() const;
    
    // 预处理接口
    void preprocess(const std::vector<std::vector<int>>& grid) override;

private:
    // 创建聚类
    void create_clusters();
    
    // 识别入口和出口点
    void identify_entrances_and_exits();
    
    // 计算抽象边
    void compute_abstract_edges();
    
    // 在抽象图上进行搜索
    std::vector<Vertex> search_abstract_graph(const Vertex& start, const Vertex& target);
    
    // 重构完整路径
    std::vector<Vertex> reconstruct_path(const std::shared_ptr<AbstractNode>& goal_node, 
                                        const Vertex& start, const Vertex& target);
    
    // 合并连续的边界点并生成出口点
    std::vector<Vertex> generate_exits_from_boundary(const std::vector<Vertex>& boundary) const;
    
    // 获取包含指定点的聚类ID
    int get_cluster_id(const Vertex& v) const;
    
    // 检查两个点是否在同一聚类内
    bool same_cluster(const Vertex& a, const Vertex& b) const;

    // 辅助：根据坐标查找cluster指针
    Cluster* find_cluster_by_position(int x, int y);
};

#endif // HPASTAR_H 