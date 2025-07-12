#include "KeyIntervalAStar.h"
#include <algorithm>
#include "../action/Action.h"
#include "../astar/AStar.h"
#include "../utilities/Log.h"

KeyIntervalAStar::KeyIntervalAStar(const Preprocess &preprocess)
    : preprocess_(preprocess)
{
}

std::vector<Vertex> KeyIntervalAStar::search(const Vertex &start, const Vertex &target)
{
    // 1. 可达性检查
    if (!utils::isPassable(preprocess_.getGrid(), start) || !utils::isPassable(preprocess_.getGrid(), target))
    {
        logger::log_info("Start or target vertex is unreachable");
        return {};
    }

    // 2. 查找起点和终点的key intervals
    auto result = findNearestKeyIntervals(start, target);
    if (result.result == ReachabilityResult::UNREACHABLE)
    {
        logger::log_info("Start or target vertex is unreachable");
        return {};
    }
    if (result.result == ReachabilityResult::DIRECT_PATH)
    {
        logger::log_info("start and target are directly reachable");
        return constructPath({start, target});
    }

    // 初始化A*搜索
    std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> openList;
    std::unordered_map<IntervalKey, double, IntervalKeyHash> gScore; // 记录g值

    auto startInterval = findContainingVerticalInterval(start);
    if(preprocess_.findKeyInterval(IntervalKey{startInterval->y, startInterval->start, startInterval->end})){
        gScore[IntervalKey{startInterval->y, startInterval->start, startInterval->end}] = 0;
    }

    // 将起点的key intervals加入openList
    for (const auto &[startKeyInterval, startDirectNeighbor] : result.startIntervals)
    {
        std::vector<Vertex> initialWaypoints = {start};
        auto [g, h] = calcGAndH(initialWaypoints, startKeyInterval, target);
        Vertex upVertex = Vertex(startKeyInterval.end, startKeyInterval.y);
        Vertex downVertex = Vertex(startKeyInterval.start, startKeyInterval.y);
        SearchNode initialNode = SearchNode(startKeyInterval, g, g + h,
                                            IntervalKey{-1, -1, -1},
                                            startDirectNeighbor,
                                            upVertex, downVertex,
                                            initialWaypoints);
        insertNode(initialNode, openList, gScore);
    }

    while (!openList.empty())
    {
        SearchNode current = openList.top();
        openList.pop();

        // 检查是否到达终点
        if (current.isTarget)
        {
            return constructPath(current.waypoints);
        }

        // 获取当前key interval
        const auto &currentInterval = preprocess_.getKeyIntervals().at(current.keyInterval);
        
        // 检查是否在终点key intervals中
        for (const auto &[targetKeyInterval, targetFromDirectNeighbor] : result.targetIntervals)
        {
            if (current.keyInterval == targetKeyInterval)
            {
                auto targetNode = handleTargetKeyInterval(current, targetFromDirectNeighbor, target);
                insertNode(targetNode, openList, gScore);
                break;
            }
        }

        // 遍历当前key interval的邻居
        for (const auto &triple : currentInterval.neighbors)
        {
            // 跳过父节点
            if (triple.neighborKeyInterval == current.parent)
                continue;
            logger::log_info("expand neighbor: [" + std::to_string(triple.neighborKeyInterval.y) + "," + std::to_string(triple.neighborKeyInterval.start) + "," + std::to_string(triple.neighborKeyInterval.end) + "] from parent: [" + std::to_string(current.keyInterval.y) + "," + std::to_string(current.keyInterval.start) + "," + std::to_string(current.keyInterval.end) + "]");
            auto neighborNode = handleNeighbor(current, triple, target);
            insertNode(neighborNode, openList, gScore);
        }
    }

    // 如果没有找到路径，返回空路径
    return {};
}

KeyIntervalAStar::NearestKeyIntervalsResult KeyIntervalAStar::findNearestKeyIntervals(const Vertex &start, const Vertex &target)
{
    logger::log_info("find nearest key intervals for start: (" + std::to_string(start.x) + "," + std::to_string(start.y) +
                     ") and target: (" + std::to_string(target.x) + "," + std::to_string(target.y) + ")");

    // 1. 查找起点的key intervals
    auto startInterval = findContainingVerticalInterval(start);
    if (!startInterval)
    {
        logger::log_info("no vertical interval found for start: " + std::to_string(start.x) + "," + std::to_string(start.y));
        return {ReachabilityResult::UNREACHABLE, {}}; // 没有找到包含vertex的vertical interval，不可达
    }

    // 2. 处理邻居链
    auto [startResult, startIntervals] = processNeighborChains(*startInterval, start, target, true);
    if (startResult == ReachabilityResult::UNREACHABLE)
    {
        logger::log_info("Start vertex is unreachable");
        return {ReachabilityResult::UNREACHABLE, {}, {}};
    }
    if (startResult == ReachabilityResult::DIRECT_PATH)
    {
        logger::log_info("start and target are directly reachable");
        return {ReachabilityResult::DIRECT_PATH, {}, {}};
    }

    // 2. 查找终点的key intervals
    auto targetInterval = findContainingVerticalInterval(target);
    if (!targetInterval)
    {
        logger::log_info("no vertical interval found for target: " + std::to_string(target.x) + "," + std::to_string(target.y));
        return {ReachabilityResult::UNREACHABLE, {}, {}}; // 没有找到包含vertex的vertical interval，不可达
    }
    auto [targetResult, targetIntervals] = processNeighborChains(*targetInterval, target, target, false);
    if (targetResult == ReachabilityResult::UNREACHABLE)
    {
        logger::log_info("Target vertex is unreachable");
        return {ReachabilityResult::UNREACHABLE, {}, {}};
    }

    return {ReachabilityResult::DONE, startIntervals, targetIntervals};
}

std::optional<VerticalInterval> KeyIntervalAStar::findContainingVerticalInterval(const Vertex &vertex) const
{
    const auto &verticalIntervals = preprocess_.getVerticalIntervals();

    // 检查vertex.y是否在有效范围内
    auto yIt = verticalIntervals.find(vertex.y);
    if (yIt == verticalIntervals.end())
    {
        return std::nullopt;
    }

    for (const auto &[_, interval] : yIt->second)
    {
        if (vertex.x >= interval.start && vertex.x <= interval.end)
        {
            return interval;
        }
    }
    return std::nullopt;
}

std::pair<KeyIntervalAStar::ReachabilityResult, std::unordered_map<IntervalKey, IntervalKey, IntervalKeyHash>> KeyIntervalAStar::processNeighborChains(const VerticalInterval &interval,
                                                                                                                                       const Vertex &vertex,
                                                                                                                                       const Vertex &target,
                                                                                                                                       bool isStart) const
{
    std::unordered_map<IntervalKey, IntervalKey, IntervalKeyHash> keyIntervals;

    // 首先检查当前interval本身是否是key interval且包含target
    IntervalKey currentKey{interval.y, interval.start, interval.end};
    if (isStart && isVertexInKeyInterval(target, currentKey))
    {
        logger::log_info("find direct path in current interval for vertex: " + std::to_string(vertex.x) + "," + std::to_string(vertex.y));
        return {ReachabilityResult::DIRECT_PATH, keyIntervals};
    }

    // 使用统一的邻居链处理方法
    if (processNeighborChain(currentKey, vertex, target, isStart, keyIntervals, true))
    {
        logger::log_info("find direct path in left neighbors for vertex: " + std::to_string(vertex.x) + "," + std::to_string(vertex.y));
        return {ReachabilityResult::DIRECT_PATH, keyIntervals};
    }

    if (processNeighborChain(currentKey, vertex, target, isStart, keyIntervals, false))
    {
        logger::log_info("find direct path in right neighbors for vertex: " + std::to_string(vertex.x) + "," + std::to_string(vertex.y));
        return {ReachabilityResult::DIRECT_PATH, keyIntervals};
    }

    // 如果找不到key intervals，说明不可达
    if (keyIntervals.empty())
    {
        logger::log_info("unreachable for vertex: " + std::to_string(vertex.x) + "," + std::to_string(vertex.y));
        return {ReachabilityResult::UNREACHABLE, keyIntervals};
    }

    return {ReachabilityResult::DONE, keyIntervals};
}

// 新增：安全的vertical interval访问
std::optional<VerticalInterval> KeyIntervalAStar::getVerticalInterval(const IntervalKey &key) const
{
    const auto &verticalIntervals = preprocess_.getVerticalIntervals();

    auto yIt = verticalIntervals.find(key.y);
    if (yIt == verticalIntervals.end())
    {
        return std::nullopt;
    }

    auto startIt = yIt->second.find(key.start);
    if (startIt == yIt->second.end())
    {
        return std::nullopt;
    }

    return startIt->second;
}

// 新增：统一的邻居链处理逻辑
bool KeyIntervalAStar::processNeighborChain(const IntervalKey &currentKey,
                                            const Vertex &vertex,
                                            const Vertex &target,
                                            bool isStart,
                                            std::unordered_map<IntervalKey, IntervalKey, IntervalKeyHash> &keyIntervals,
                                            bool isLeftDirection) const
{
    const auto &currentInterval = preprocess_.getVerticalIntervals().at(currentKey.y).at(currentKey.start);
    const auto &neighbors = isLeftDirection ? currentInterval.leftNeighbors : currentInterval.rightNeighbors;
    for (const auto &neighbor : neighbors)
    {
        // logger::log_info("process neighbor: " + std::to_string(neighbor.y) + "," + std::to_string(neighbor.start) + "," + std::to_string(neighbor.end) + " for vertex: " + std::to_string(vertex.x) + "," + std::to_string(vertex.y));
        //  检查neighbor本身是否是key interval
        if (isStart && isVertexInKeyInterval(target, neighbor))
        {
            keyIntervals.clear();
            logger::log_info("find target vertex: " + std::to_string(neighbor.y) + "," + std::to_string(neighbor.start) + "," + std::to_string(neighbor.end) + " for vertex: " + std::to_string(vertex.x) + "," + std::to_string(vertex.y));
            return true; // 找到target vertex
        }

        if (preprocess_.findKeyInterval(neighbor))
        {
            logger::log_info("find key interval: " + std::to_string(neighbor.y) + "," + std::to_string(neighbor.start) + "," + std::to_string(neighbor.end) + " for vertex: " + std::to_string(vertex.x) + "," + std::to_string(vertex.y));
            keyIntervals[neighbor] = currentKey;
            break; // 找到关键区间，退出循环
        }

        // 安全地获取vertical interval
        auto currentIntervalOpt = getVerticalInterval(neighbor);
        if (!currentIntervalOpt)
        {
            continue; // 跳过无效的neighbor
        }

        VerticalInterval currentInterval = *currentIntervalOpt;
        IntervalKey directNeighbor = neighbor; // 记录路径上的直接邻居

        // 沿着邻居链查找key interval
        while (true)
        {
            const auto &neighborSet = isLeftDirection ? currentInterval.leftNeighbors : currentInterval.rightNeighbors;

            if (neighborSet.empty())
            {
                break; // 没有更多邻居，退出循环
            }

            const auto &nextNeighbor = *neighborSet.begin();
            logger::log_info("next neighbor: " + std::to_string(nextNeighbor.y) + "," + std::to_string(nextNeighbor.start) + "," + std::to_string(nextNeighbor.end) + " for vertex: " + std::to_string(vertex.x) + "," + std::to_string(vertex.y));
            if (isStart && isVertexInKeyInterval(target, nextNeighbor))
            {
                keyIntervals.clear();
                logger::log_info("find target vertex: " + std::to_string(nextNeighbor.y) + "," + std::to_string(nextNeighbor.start) + "," + std::to_string(nextNeighbor.end) + " for vertex: " + std::to_string(vertex.x) + "," + std::to_string(vertex.y));
                return true; // 找到target vertex
            }

            if (preprocess_.findKeyInterval(nextNeighbor))
            {
                logger::log_info("find key interval: " + std::to_string(nextNeighbor.y) + "," + std::to_string(nextNeighbor.start) + "," + std::to_string(nextNeighbor.end) + " for vertex: " + std::to_string(vertex.x) + "," + std::to_string(vertex.y));
                keyIntervals[nextNeighbor] = directNeighbor;
                break; // 找到关键区间，退出循环
            }

            // 安全地获取下一个interval
            auto nextIntervalOpt = getVerticalInterval(nextNeighbor);
            if (!nextIntervalOpt)
            {
                break; // 无效的neighbor，退出循环
            }
            directNeighbor = nextNeighbor;
            currentInterval = *nextIntervalOpt;
        }
    }

    return false; // 没有找到target vertex
}

bool KeyIntervalAStar::isVertexInKeyInterval(const Vertex &vertex, const IntervalKey &key) const
{
    return vertex.y == key.y &&
           vertex.x >= key.start &&
           vertex.x <= key.end;
}

KeyIntervalAStar::SearchNode KeyIntervalAStar::handleNeighbor(const KeyIntervalAStar::SearchNode &current,
                                      const NeighborTriple &neighborTriple,
                                      const Vertex &target)
{
    // 检查是否有transition vertex
    auto transitionVertex = findTransitionVertex(current.keyInterval, current.fromDirectNeighbor, neighborTriple.currentDirectNeighbor);
    if (transitionVertex.has_value())
    {
        logger::log_info("find transition vertex: " + std::to_string(transitionVertex.value().x) + "," + std::to_string(transitionVertex.value().y));
        return directTransition(current, neighborTriple, transitionVertex.value(), target);
    }
    
    // 处理key points
    return handleUpDownVertex(current, neighborTriple, target);
}

KeyIntervalAStar::SearchNode KeyIntervalAStar::directTransition(const KeyIntervalAStar::SearchNode &current,
                                        const NeighborTriple &neighborTriple,
                                        const Vertex &transitionVertex,
                                        const Vertex &target)
{
    std::vector<Vertex> newWaypoints = current.waypoints;
    
    if (checkAndAddUpDownVertex(transitionVertex, current.waypoints.back(), newWaypoints, current))
    {
        logger::log_info("up/down vertex added: " + std::to_string(newWaypoints.back().x) + "," + std::to_string(newWaypoints.back().y));
        newWaypoints.push_back(transitionVertex);
        auto [g, h] = calcGAndH(newWaypoints, neighborTriple.neighborKeyInterval, target);

        KeyIntervalAStar::SearchNode newNode = KeyIntervalAStar::SearchNode(neighborTriple.neighborKeyInterval, g, g + h,
                                        current.keyInterval,
                                        neighborTriple.neighborDirectNeighbor,
                                        Vertex(-1, -1), Vertex(-1, -1),
                                        newWaypoints);
        return newNode;
    }
    else
    {
        newWaypoints.push_back(transitionVertex);
        auto [g, h] = calcGAndH(newWaypoints, neighborTriple.neighborKeyInterval, target);

        KeyIntervalAStar::SearchNode newNode = KeyIntervalAStar::SearchNode(neighborTriple.neighborKeyInterval, g, g + h,
                                            current.keyInterval,
                                            neighborTriple.neighborDirectNeighbor,
                                            Vertex(-1, -1), Vertex(-1, -1),
                                            newWaypoints);
        return newNode;
    }
}

KeyIntervalAStar::SearchNode KeyIntervalAStar::handleUpDownVertex(const KeyIntervalAStar::SearchNode &current,
                                          const NeighborTriple &neighborTriple,
                                          const Vertex &target)
{
    const auto &neighborInterval = preprocess_.getKeyIntervals().at(neighborTriple.neighborKeyInterval);

    Vertex tempUpVertex = current.upVertex;
    Vertex tempDownVertex = current.downVertex;
    bool shouldBreak = false;
    std::vector<Vertex> newWaypoints = current.waypoints;

    // 检查邻居interval中的key points
    for (const auto &keyPoint : neighborInterval.getVerticalKeyPoints())
    {
        if (keyPoint.direction == Preprocess::Direction::UP)
        {
            if (current.downVertex.x != -1 &&
                keyPoint.point.x < current.downVertex.x &&
                current.waypoints.back().x < current.downVertex.x)
            {
                newWaypoints.push_back(current.downVertex);
                newWaypoints.push_back(keyPoint.point);
                shouldBreak = true;
                break;
            }
            if (tempUpVertex.x == -1 || keyPoint.point.x < tempUpVertex.x)
            {
                tempUpVertex = keyPoint.point;
            }
        }
        else if (keyPoint.direction == Preprocess::Direction::DOWN)
        {
            if (current.upVertex.x != -1 &&
                keyPoint.point.x > current.upVertex.x &&
                current.waypoints.back().x > current.upVertex.x)
            {
                newWaypoints.push_back(current.upVertex);
                newWaypoints.push_back(keyPoint.point);
                shouldBreak = true;
                break;
            }
            if (tempDownVertex.x == -1 || keyPoint.point.x > tempDownVertex.x)
            {
                tempDownVertex = keyPoint.point;
            }
        }
    }

    Vertex newUpVertex = shouldBreak ? Vertex(-1, -1) : tempUpVertex;
    Vertex newDownVertex = shouldBreak ? Vertex(-1, -1) : tempDownVertex;

    IntervalKey evaluationInterval = {neighborTriple.neighborKeyInterval.y, 
                                    newDownVertex.x == -1 ? neighborTriple.neighborKeyInterval.start : newDownVertex.x,
                                    newUpVertex.x == -1 ? neighborTriple.neighborKeyInterval.end : newUpVertex.x};
    logger::log_info("evaluation interval: " + std::to_string(evaluationInterval.y) + "," + std::to_string(evaluationInterval.start) + "," + std::to_string(evaluationInterval.end));
    auto [g, h] = calcGAndH(newWaypoints, evaluationInterval, target);

    logger::log_info("g: " + std::to_string(g) + " h: " + std::to_string(h));
    logger::log_info("come from: " + std::to_string(neighborTriple.neighborDirectNeighbor.y) + "," + std::to_string(neighborTriple.neighborDirectNeighbor.start) + "," + std::to_string(neighborTriple.neighborDirectNeighbor.end));
    logger::log_info("up vertex: " + std::to_string(newUpVertex.x) + "," + std::to_string(newUpVertex.y));
    logger::log_info("down vertex: " + std::to_string(newDownVertex.x) + "," + std::to_string(newDownVertex.y));
    KeyIntervalAStar::SearchNode newNode = KeyIntervalAStar::SearchNode(neighborTriple.neighborKeyInterval, g, g + h,
                                        current.keyInterval,
                                        neighborTriple.neighborDirectNeighbor,
                                        newUpVertex, newDownVertex,
                                        newWaypoints);
    return newNode;
}

KeyIntervalAStar::SearchNode KeyIntervalAStar::handleTargetKeyInterval(const KeyIntervalAStar::SearchNode &current,
                                               const IntervalKey &targetFromDirectNeighbor,
                                               const Vertex &target)
{
    logger::log_info("handle target key interval: " + std::to_string(current.keyInterval.y) + "," + std::to_string(current.keyInterval.start) + "," + std::to_string(current.keyInterval.end));
    std::vector<Vertex> finalWaypoints = current.waypoints;
    double g = current.g;

    const auto &endKeyInterval = preprocess_.getKeyIntervals().at(current.keyInterval);
    logger::log_info("current from direct neighbor: " + std::to_string(current.fromDirectNeighbor.y) + "," + std::to_string(current.fromDirectNeighbor.start) + "," + std::to_string(current.fromDirectNeighbor.end));
    logger::log_info("target from direct neighbor: " + std::to_string(targetFromDirectNeighbor.y) + "," + std::to_string(targetFromDirectNeighbor.start) + "," + std::to_string(targetFromDirectNeighbor.end));

    auto transitionVertexOpt = findTransitionVertex(current.keyInterval, current.fromDirectNeighbor, targetFromDirectNeighbor);

    if (transitionVertexOpt.has_value())
    {
        Vertex transitionVertex = transitionVertexOpt.value();
        logger::log_info("find transition vertex: " + std::to_string(transitionVertex.x) + "," + std::to_string(transitionVertex.y));

        if(checkAndAddUpDownVertex(transitionVertex, current.waypoints.back(), finalWaypoints, current))
        {
            g += calculateHeuristicDistance(*(finalWaypoints.rbegin()+1), finalWaypoints.back());
            g += calculateHeuristicDistance(finalWaypoints.back(), transitionVertex);
            g += calculateHeuristicDistance(transitionVertex, target);

            finalWaypoints.push_back(transitionVertex);
            finalWaypoints.push_back(target);

            KeyIntervalAStar::SearchNode targetNode = KeyIntervalAStar::SearchNode(IntervalKey{-1, -1, -1}, g, g,
                                               current.keyInterval,
                                               IntervalKey{-1, -1, -1},
                                               Vertex(-1, -1), Vertex(-1, -1),
                                               finalWaypoints, true);
            return targetNode;
        }else{
            g += calculateHeuristicDistance(finalWaypoints.back(), transitionVertex);
            g += calculateHeuristicDistance(transitionVertex, target);
            finalWaypoints.push_back(transitionVertex);
            finalWaypoints.push_back(target);

            KeyIntervalAStar::SearchNode targetNode = KeyIntervalAStar::SearchNode(IntervalKey{-1, -1, -1}, g, g,
                                               current.keyInterval,
                                               IntervalKey{-1, -1, -1},
                                               Vertex(-1, -1), Vertex(-1, -1),
                                               finalWaypoints, true);
            return targetNode;
        }


        logger::log_info("no up/down vertex found before transition vertex for target");
    }

    // 检查最后一个waypoint到target vertex之间是否还有关键点
    Vertex lastWaypoint = current.waypoints.back();

    if(checkAndAddUpDownVertex(target, lastWaypoint, finalWaypoints, current))
    {
        g += calculateHeuristicDistance(lastWaypoint, finalWaypoints.back());
    }

    g += calculateHeuristicDistance(finalWaypoints.back(), target);
    finalWaypoints.push_back(target);
    logger::log_info("final waypoints: " + logger::vectorToString(finalWaypoints));

    KeyIntervalAStar::SearchNode targetNode(IntervalKey{-1, -1, -1}, g, g, current.keyInterval, IntervalKey{-1, -1, -1},
                          current.upVertex, current.downVertex, finalWaypoints, true);
    logger::log_info("push target node");
    return targetNode;
}

std::pair<double, double> KeyIntervalAStar::calcGAndH(const std::vector<Vertex> &waypoints,
                                                      const IntervalKey &evaluationInterval,
                                                      const Vertex &target) const
{
    double gValue = 0.0;
    double hValue = 0.0;

    logger::log_info("calculate g and h for evaluation interval: " + std::to_string(evaluationInterval.y) + "," + std::to_string(evaluationInterval.start) + "," + std::to_string(evaluationInterval.end));
    logger::log_info("waypoints: " + logger::vectorToString(waypoints));

    // 计算mustPassPoints中相邻点之间的代价
    for (size_t i = 1; i < waypoints.size(); ++i)
    {
        gValue += calculateHeuristicDistance(waypoints[i - 1], waypoints[i]);
    }

    // 计算最后一段的代价
    if (!isVertexInKeyInterval(waypoints.back(), evaluationInterval))
    {
        logger::log_info("calculate g and h with last waypoint not in current key interval: " + std::to_string(waypoints.back().x) + "," + std::to_string(waypoints.back().y));
        // 否则根据目标vertex的位置计算代价
        if (target.x <= evaluationInterval.start)
        {
            Vertex evaluationVertex = Vertex(evaluationInterval.start, evaluationInterval.y);
            // 如果目标x小于等于interval的start
            gValue += calculateHeuristicDistance(waypoints.back(), evaluationVertex);
            hValue = calculateHeuristicDistance(evaluationVertex, target);
        }
        else if (target.x >= evaluationInterval.end)
        {
            Vertex evaluationVertex = Vertex(evaluationInterval.end, evaluationInterval.y);
            // 如果目标x大于等于interval的end
            gValue += calculateHeuristicDistance(waypoints.back(), evaluationVertex);
            hValue = calculateHeuristicDistance(evaluationVertex, target);
        }
        else
        {
            Vertex evaluationVertex = Vertex(target.x, evaluationInterval.y);
            // 如果目标x在interval的范围内
            gValue += calculateHeuristicDistance(waypoints.back(), evaluationVertex);
            hValue = calculateHeuristicDistance(evaluationVertex, target);
        }
    }else{
        logger::log_info("calculate g and h with last waypoint in current key interval: " + std::to_string(waypoints.back().x) + "," + std::to_string(waypoints.back().y));
        hValue = calculateHeuristicDistance(waypoints.back(), target);
    }

    return std::make_pair(gValue, hValue);
}

std::vector<Vertex> KeyIntervalAStar::constructPath(std::vector<Vertex> waypoints) const
{
    logger::log_info("construct path with waypoints: " + logger::vectorToString(waypoints));
    // 使用mustPassPoints重建完整路径
    std::vector<Vertex> path;
    if (waypoints.empty())
        return path;
    const auto &grid = preprocess_.getGrid();
    for (size_t i = 0; i < waypoints.size() - 1; ++i)
    {
        // 调用A*算法获取两个mustPassPoint之间的路径，去除首点避免重复
        Vertex start = waypoints[i];
        Vertex end = waypoints[i + 1];

        std::vector<Vertex> directions;
        if (end.x > start.x)
            directions.push_back(Vertex(1, 0));
        else if (end.x < start.x)
            directions.push_back(Vertex(-1, 0));
        if (end.y > start.y)
            directions.push_back(Vertex(0, 1));
        else if (end.y < start.y)
            directions.push_back(Vertex(0, -1));

        auto subPath = a_star(start, end, grid, directions);
        if (!subPath.empty())
        {
            if (i > 0 && !path.empty())
                subPath.erase(subPath.begin());
            path.insert(path.end(), subPath.begin(), subPath.end());
        }
        else
        {
            // 若A*失败
            logger::log_error("A* failed with waypoints: " + logger::vectorToString(waypoints));
            return {};
        }
    }
    return path;
}

void KeyIntervalAStar::insertNode(const SearchNode& node,
                                 std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>>& openList,
                                 std::unordered_map<IntervalKey, double, IntervalKeyHash>& gScore)
{
    // 对于目标节点，直接插入到openList中，不进行g值比较
    if (node.isTarget)
    {
        openList.push(node);
        return;
    }
    
    // 检查g值是否更优
    if (gScore.find(node.keyInterval) == gScore.end() || node.g < gScore[node.keyInterval])
    {
        gScore[node.keyInterval] = node.g;
        openList.push(node);
        logger::log_info("push new node: " + std::to_string(node.keyInterval.y) + "," + std::to_string(node.keyInterval.start) + "," + std::to_string(node.keyInterval.end));
    }
}

bool KeyIntervalAStar::checkAndAddUpDownVertex(const Vertex& targetVertex, 
                                              const Vertex& lastWaypoint,
                                              std::vector<Vertex>& waypoints,
                                              const SearchNode& current)
{
    if (current.downVertex.x != -1 &&
        targetVertex.x < current.downVertex.x &&
        lastWaypoint.x < current.downVertex.x)
    {
        waypoints.push_back(current.downVertex);
        return true;
    }

    if (current.upVertex.y != -1 &&
        targetVertex.x > current.upVertex.x &&
        lastWaypoint.x > current.upVertex.x)
    {
        waypoints.push_back(current.upVertex);
        return true;
    }
    return false;
}

std::optional<Vertex> KeyIntervalAStar::findTransitionVertex(const IntervalKey& keyInterval,
                                                            const IntervalKey& directNeighbor1,
                                                            const IntervalKey& directNeighbor2) const
{
    const auto &endKeyInterval = preprocess_.getKeyIntervals().at(keyInterval);
    
    auto it = endKeyInterval.transitionVertices.find(directNeighbor1);
    if (it != endKeyInterval.transitionVertices.end())
    {
        auto it2 = it->second.find(directNeighbor2);
        if (it2 != it->second.end())
        {
            return it2->second;
        }
    }
    
    return std::nullopt;
}
