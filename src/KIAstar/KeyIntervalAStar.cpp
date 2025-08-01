#include "KeyIntervalAStar.h"
#include <algorithm>
#include <chrono>
#include <windows.h>
#include "../action/Action.h"
#include "../astar/AStar.h"
#include "../utilities/Log.h"
#include "../utilities/MemoryUtility.h"
#include "../heuristic/Heuristic.h"

KeyIntervalAStar::KeyIntervalAStar(const Preprocess &preprocess, const std::string &name)
    : preprocess_(&preprocess), name_(name)
{
    // calculate memory usage of preprocessing
    preprocess_memory_usage_ = preprocess.getMemoryUsage();
    preprocess_time_ = preprocess.getPreprocessTime();
}

// new: default constructor
KeyIntervalAStar::KeyIntervalAStar(const std::string &name)
    : preprocess_(nullptr), name_(name)
{
}

// new: preprocessing method
void KeyIntervalAStar::preprocess(const std::vector<std::vector<int>>& grid) {
    // create new Preprocess object
    preprocess_ptr_ = std::make_unique<Preprocess>(grid);
    preprocess_ptr_->preprocess();
    
    // update pointer and statistics
    preprocess_ = preprocess_ptr_.get();
    preprocess_memory_usage_ = preprocess_->getMemoryUsage();
    preprocess_time_ = preprocess_->getPreprocessTime();
}

std::vector<Vertex> KeyIntervalAStar::search(const Vertex &start, const Vertex &target)
{
    // check if preprocessing is done
    if (!preprocess_) {
        throw std::runtime_error("Preprocess not initialized. Call preprocess() first.");
    }
    
    // reset expanded nodes count and time
    expanded_nodes_ = 0;
    search_time_ = 0.0;
    search_memory_increase_ = 0;

    // record memory usage before search
    size_t memory_before = memory_utils::get_current_memory_usage();
    
    // use QueryPerformanceCounter to measure time
    LARGE_INTEGER freq, t_start, t_end;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&t_start);

    // 1. check if reachable
    if (!utils::isPassable(preprocess_->getGrid(), start) || !utils::isPassable(preprocess_->getGrid(), target))
    {
        QueryPerformanceCounter(&t_end);
        search_time_ = static_cast<double>(t_end.QuadPart - t_start.QuadPart) / freq.QuadPart;
        return {};
    }

    auto queryResult = queryKeyIntervals(start, target);
    if (queryResult.directlyReachable)
    {
        auto path = constructPath({start, target});
        QueryPerformanceCounter(&t_end);
        search_time_ = static_cast<double>(t_end.QuadPart - t_start.QuadPart) / freq.QuadPart;
        return path;
    }

    // initialize A* search
    std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> openList;
    std::unordered_map<IntervalKey, double, IntervalKeyHash> gScore; // record g value

    initializeStartNode(start, target, queryResult, openList, gScore);

    while (!openList.empty())
    {
        SearchNode current = openList.top();
        openList.pop();
        //logger::log_info("current key interval: " + std::to_string(current.keyInterval.has_value() ? current.keyInterval.value().y : -1) + "," + std::to_string(current.keyInterval.has_value() ? current.keyInterval.value().start : -1) + "," + std::to_string(current.keyInterval.has_value() ? current.keyInterval.value().end : -1));
        //logger::log_info("current up vertex: " + (current.upVertex ? std::to_string(current.upVertex.value().x) + "," + std::to_string(current.upVertex.value().y) : "null"));
        //logger::log_info("current down vertex: " + (current.downVertex ? std::to_string(current.downVertex.value().x) + "," + std::to_string(current.downVertex.value().y) : "null"));
        //logger::log_info("from parent: " + (current.parent ? std::to_string(current.parent.value().y) + "," + std::to_string(current.parent.value().start) + "," + std::to_string(current.parent.value().end) : "null"));
        //logger::log_info("current waypoints: " + //logger::vectorToString(current.waypoints));
        //logger::log_info("g: " + std::to_string(current.g));
        //logger::log_info("h: " + std::to_string(current.f - current.g));

        // check if reached target
        if (current.isTarget)
        {
            expanded_nodes_++;
            QueryPerformanceCounter(&t_end);
            search_time_ = static_cast<double>(t_end.QuadPart - t_start.QuadPart) / freq.QuadPart;
            
            // record memory usage after search
            size_t memory_after = memory_utils::get_current_memory_usage();
            search_memory_increase_ = memory_utils::calculate_memory_increase(memory_before, memory_after);
            
            return constructPath(current.waypoints);
        }

        // check if in target key intervals
        if (handleTargetKeyIntervals(current, target, queryResult, openList, gScore))
        {
            continue;
        }
        
        // get current key interval
        const auto &currentInterval = preprocess_->getKeyIntervals().at(current.keyInterval.value());

        // traverse neighbors of current key interval
        for (const auto &triple : currentInterval.neighbors)
        {
            // skip parent node
            if (triple.neighborKeyInterval == current.parent)
                continue;
            //logger::log_info("handle neighbor: " + std::to_string(triple.neighborKeyInterval.y) + "," + std::to_string(triple.neighborKeyInterval.start) + "," + std::to_string(triple.neighborKeyInterval.end));
            auto neighborNode = handleNeighbor(current, triple, target);
            insertNode(neighborNode, openList, gScore);
            expanded_nodes_++; // increase expanded nodes count
        }
    }

    // if no path found, return empty path
    QueryPerformanceCounter(&t_end);
    search_time_ = static_cast<double>(t_end.QuadPart - t_start.QuadPart) / freq.QuadPart;
    
    // record memory usage after search
    size_t memory_after = memory_utils::get_current_memory_usage();
    search_memory_increase_ = memory_utils::calculate_memory_increase(memory_before, memory_after);
    
    return {};
}

KeyIntervalAStar::KeyIntervalQueryResult KeyIntervalAStar::queryKeyIntervals(const Vertex &start, const Vertex &target) const
{
    KeyIntervalQueryResult result;
    auto startIntervalOpt = preprocess_->findContainingVerticalInterval(start);
    auto targetIntervalOpt = preprocess_->findContainingVerticalInterval(target);
    if (!startIntervalOpt || !targetIntervalOpt)
    {
        result.directlyReachable = false;
        return result;
    }
    const auto &startInterval = *startIntervalOpt;
    const auto &targetInterval = *targetIntervalOpt;

    IntervalKey startKey{startInterval.y, startInterval.start, startInterval.end};
    IntervalKey targetKey{targetInterval.y, targetInterval.start, targetInterval.end};

    // set start and target key intervals
    if (preprocess_->findKeyInterval(startKey))
    {
        result.startK = startKey;
    }
    if (preprocess_->findKeyInterval(targetKey))
    {
        result.targetK = targetKey;
    }

    const auto &startLeft = startInterval.leftKeyInterval;
    const auto &startRight = startInterval.rightKeyInterval;
    const auto &targetLeft = targetInterval.leftKeyInterval;
    const auto &targetRight = targetInterval.rightKeyInterval;

    // check if directly reachable
    result.directlyReachable =
        (result.startK && result.targetK && result.startK.value() == result.targetK.value()) ||
        (result.startK && targetLeft && targetLeft->first == result.startK.value()) ||
        (result.startK && targetRight && targetRight->first == result.startK.value()) ||
        (result.targetK && startLeft && startLeft->first == result.targetK.value()) ||
        (result.targetK && startRight && startRight->first == result.targetK.value()) ||
        (!result.startK && !result.targetK && startLeft == targetLeft && startRight == targetRight) ||
        (!result.startK && !result.targetK && !startLeft && !startRight) ||
        (!result.startK && !result.targetK && !targetLeft && !targetRight);

    // only need A* key interval set when not directly reachable
    if (!result.directlyReachable)
    {
        result.startLeft = startLeft;
        result.startRight = startRight;
        result.targetLeft = targetLeft;
        result.targetRight = targetRight;
    }
    return result;
}

void KeyIntervalAStar::initializeStartNode(const Vertex &start, const Vertex &target,
                                           const KeyIntervalQueryResult &queryResult,
                                           std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> &openList,
                                           std::unordered_map<IntervalKey, double, IntervalKeyHash> &gScore)
{
    if (queryResult.startK)
    {
        gScore[queryResult.startK.value()] = 0.0;
        for (NeighborTriple triple : preprocess_->getKeyIntervals().at(queryResult.startK.value()).neighbors) {
            auto startKeyInterval = triple.neighborKeyInterval;
            auto startDirectNeighbor = triple.neighborDirectNeighbor;
            std::vector<Vertex> initialWaypoints = {start};
            auto [g, h] = calcGAndH(initialWaypoints, startKeyInterval, target);
            std::optional<Vertex> up = preprocess_->getKeyIntervals().at(startKeyInterval).getUpVertex();
            std::optional<Vertex> down = preprocess_->getKeyIntervals().at(startKeyInterval).getDownVertex();
            SearchNode initialNode = SearchNode(startKeyInterval, g, g + h,
                                                std::nullopt,
                                                startDirectNeighbor,
                                                up, down,
                                                initialWaypoints);
            insertNode(initialNode, openList, gScore);
        }
    }

    // start not in key intervals, use left and right neighbors
    if (queryResult.startLeft)
    {
        const auto &[startKeyInterval, startDirectNeighbor] = queryResult.startLeft.value();
        std::vector<Vertex> initialWaypoints = {start};
        auto [g, h] = calcGAndH(initialWaypoints, startKeyInterval, target);
        std::optional<Vertex> up = preprocess_->getKeyIntervals().at(startKeyInterval).getUpVertex();
        std::optional<Vertex> down = preprocess_->getKeyIntervals().at(startKeyInterval).getDownVertex();
        SearchNode initialNode = SearchNode(startKeyInterval, g, g + h,
                                            std::nullopt,
                                            startDirectNeighbor,
                                            up, down,
                                            initialWaypoints);
        insertNode(initialNode, openList, gScore);
    }
    if (queryResult.startRight)
    {
        const auto &[startKeyInterval, startDirectNeighbor] = queryResult.startRight.value();
        std::vector<Vertex> initialWaypoints = {start};
        auto [g, h] = calcGAndH(initialWaypoints, startKeyInterval, target);
        std::optional<Vertex> up = preprocess_->getKeyIntervals().at(startKeyInterval).getUpVertex();
        std::optional<Vertex> down = preprocess_->getKeyIntervals().at(startKeyInterval).getDownVertex();
        SearchNode initialNode = SearchNode(startKeyInterval, g, g + h,
                                            std::nullopt,
                                            startDirectNeighbor,
                                            up, down,
                                            initialWaypoints);
        insertNode(initialNode, openList, gScore);
    }
}

KeyIntervalAStar::SearchNode KeyIntervalAStar::handleNeighbor(const KeyIntervalAStar::SearchNode &current,
                                                              const NeighborTriple &neighborTriple,
                                                              const Vertex &target)
{
    // check if has transition vertex
    auto transitionVertex = preprocess_->findTransitionVertex(current.keyInterval.value(), current.fromDirectNeighbor.value(), neighborTriple.currentDirectNeighbor);
    if (transitionVertex.has_value())
    {
        //logger::log_info("find transition vertex: " + std::to_string(transitionVertex.value().x) + "," + std::to_string(transitionVertex.value().y));
        return directTransition(current, neighborTriple, transitionVertex.value(), target);
    }

    // handle key points
    return handleUpDownVertex(current, neighborTriple, target);
}

KeyIntervalAStar::SearchNode KeyIntervalAStar::directTransition(const KeyIntervalAStar::SearchNode &current,
                                                                const NeighborTriple &neighborTriple,
                                                                const Vertex &transitionVertex,
                                                                const Vertex &target)
{
    std::vector<Vertex> newWaypoints = current.waypoints;
    KeyInterval neighborKeyInterval = preprocess_->getKeyIntervals().at(neighborTriple.neighborKeyInterval);


    if (checkAndAddUpDownVertex(transitionVertex, current.waypoints.back(), newWaypoints, current))
    {
        //logger::log_info("up/down vertex added: " + std::to_string(newWaypoints.back().x) + "," + std::to_string(newWaypoints.back().y));
        newWaypoints.push_back(transitionVertex);
        auto [g, h] = calcGAndH(newWaypoints, neighborTriple.neighborKeyInterval, target);

        std::optional<Vertex> upVertex = neighborKeyInterval.getUpVertex();
        std::optional<Vertex> downVertex = neighborKeyInterval.getDownVertex();
        KeyIntervalAStar::SearchNode newNode = KeyIntervalAStar::SearchNode(neighborTriple.neighborKeyInterval, g, g + h,
                                                                            current.keyInterval,
                                                                            neighborTriple.neighborDirectNeighbor,
                                                                            upVertex, downVertex,
                                                                            newWaypoints);
        return newNode;
    }
    else
    {
        newWaypoints.push_back(transitionVertex);
        auto [g, h] = calcGAndH(newWaypoints, neighborTriple.neighborKeyInterval, target);

        std::optional<Vertex> upVertex = neighborKeyInterval.getUpVertex();
        std::optional<Vertex> downVertex = neighborKeyInterval.getDownVertex();
        KeyIntervalAStar::SearchNode newNode = KeyIntervalAStar::SearchNode(neighborTriple.neighborKeyInterval, g, g + h,
                                                                            current.keyInterval,
                                                                            neighborTriple.neighborDirectNeighbor,
                                                                            upVertex, downVertex,
                                                                            newWaypoints);
        return newNode;
    }
}

KeyIntervalAStar::SearchNode KeyIntervalAStar::handleUpDownVertex(const KeyIntervalAStar::SearchNode &current,
                                                                  const NeighborTriple &neighborTriple,
                                                                  const Vertex &target)
{
    //logger::log_info("handle up/down vertex");
    const auto &neighborInterval = preprocess_->getKeyIntervals().at(neighborTriple.neighborKeyInterval);

    std::optional<Vertex> tempUpVertex = current.upVertex;
    std::optional<Vertex> tempDownVertex = current.downVertex;
    bool shouldBreak = false;
    std::vector<Vertex> newWaypoints = current.waypoints;

    if (current.downVertex &&
        neighborInterval.getEnd() < current.downVertex.value().x &&
        current.waypoints.back().x < current.downVertex.value().x)
    {
        newWaypoints.push_back(current.downVertex.value());
        newWaypoints.push_back(Vertex(neighborInterval.getEnd(), neighborInterval.getY()));
        shouldBreak = true;
        //logger::log_info("new waypoints: " + //logger::vectorToString(newWaypoints));
    }

    if (!shouldBreak && 
        current.upVertex &&
        neighborInterval.getStart() > current.upVertex.value().x &&
        current.waypoints.back().x > current.upVertex.value().x)
    {
        newWaypoints.push_back(current.upVertex.value());
        newWaypoints.push_back(Vertex(neighborInterval.getStart(), neighborInterval.getY()));
        shouldBreak = true;
        //logger::log_info("new waypoints: " + //logger::vectorToString(newWaypoints));
    }

    if (!shouldBreak) {
        std::optional<Vertex> upV = neighborInterval.getUpVertex();
        if (upV) {
            if (!tempUpVertex || upV.value().x < tempUpVertex.value().x){
                tempUpVertex = upV;
                //logger::log_info("update up vertex: " + std::to_string(tempUpVertex.value().x) + "," + std::to_string(tempUpVertex.value().y));
            }
        }
        std::optional<Vertex> downV = neighborInterval.getDownVertex();
        if (downV) {
            if (!tempDownVertex || downV.value().x > tempDownVertex.value().x){
                tempDownVertex = downV;
                //logger::log_info("update down vertex: " + std::to_string(tempDownVertex.value().x) + "," + std::to_string(tempDownVertex.value().y));
            }
        }
    }

    std::optional<Vertex> newUpVertex = shouldBreak ? std::nullopt : tempUpVertex;
    std::optional<Vertex> newDownVertex = shouldBreak ? std::nullopt : tempDownVertex;

    auto [g, h] = calcGAndH(newWaypoints, neighborTriple.neighborKeyInterval, target);
    
    KeyIntervalAStar::SearchNode newNode = KeyIntervalAStar::SearchNode(neighborTriple.neighborKeyInterval, g, g + h,
                                                                        current.keyInterval,
                                                                        neighborTriple.neighborDirectNeighbor,
                                                                        newUpVertex, newDownVertex,
                                                                        newWaypoints);
    return newNode;
}

bool KeyIntervalAStar::handleTargetKeyIntervals(const SearchNode &current, const Vertex &target,
                                                const KeyIntervalQueryResult &queryResult,
                                                std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> &openList,
                                                std::unordered_map<IntervalKey, double, IntervalKeyHash> &gScore)
{
    // check if in target key intervals
    if (queryResult.targetK && current.keyInterval.value() == queryResult.targetK.value())
    {
        // target in key intervals
        auto targetNode = handleTargetKeyInterval(current, queryResult.targetK.value(), target);
        insertNode(targetNode, openList, gScore);
        expanded_nodes_++;
        return true;
    }
    else
    {
        // target not in key intervals, check left and right neighbors
        if (queryResult.targetLeft)
        {
            const auto &[targetKeyInterval, targetFromDirectNeighbor] = queryResult.targetLeft.value();
            if (current.keyInterval.value() == targetKeyInterval)
            {
                auto targetNode = handleTargetKeyInterval(current, targetFromDirectNeighbor, target);
                insertNode(targetNode, openList, gScore);
                expanded_nodes_++;
                return true;
            }
        }
        if (queryResult.targetRight)
        {
            const auto &[targetKeyInterval, targetFromDirectNeighbor] = queryResult.targetRight.value();
            if (current.keyInterval.value() == targetKeyInterval)
            {
                auto targetNode = handleTargetKeyInterval(current, targetFromDirectNeighbor, target);
                insertNode(targetNode, openList, gScore);
                expanded_nodes_++;
                return true;
            }
        }
    }
    return false;
}

KeyIntervalAStar::SearchNode KeyIntervalAStar::handleTargetKeyInterval(const KeyIntervalAStar::SearchNode &current,
                                                                       const IntervalKey &targetFromDirectNeighbor,
                                                                       const Vertex &target)
{
    //logger::log_info("handle target key interval: " + std::to_string(current.keyInterval.value().y) + "," + std::to_string(current.keyInterval.value().start) + "," + std::to_string(current.keyInterval.value().end));
    std::vector<Vertex> finalWaypoints = current.waypoints;

    const auto &endKeyInterval = preprocess_->getKeyIntervals().at(current.keyInterval.value());
    //logger::log_info("current from direct neighbor: " + std::to_string(current.fromDirectNeighbor.value().y) + "," + std::to_string(current.fromDirectNeighbor.value().start) + "," + std::to_string(current.fromDirectNeighbor.value().end));
    //logger::log_info("target from direct neighbor: " + std::to_string(targetFromDirectNeighbor.y) + "," + std::to_string(targetFromDirectNeighbor.start) + "," + std::to_string(targetFromDirectNeighbor.end));

    auto transitionVertexOpt = preprocess_->findTransitionVertex(current.keyInterval.value(), current.fromDirectNeighbor.value(), targetFromDirectNeighbor);

    if (transitionVertexOpt.has_value())
    {
        Vertex transitionVertex = transitionVertexOpt.value();
        //logger::log_info("find transition vertex: " + std::to_string(transitionVertex.x) + "," + std::to_string(transitionVertex.y));

        if (checkAndAddUpDownVertex(transitionVertex, current.waypoints.back(), finalWaypoints, current))
        {
            finalWaypoints.push_back(transitionVertex);
            finalWaypoints.push_back(target);

            double g = 0.0;
            for (size_t i = 1; i < finalWaypoints.size(); ++i)
            {
                g += heuristic(finalWaypoints[i - 1], finalWaypoints[i]);
            }

            KeyIntervalAStar::SearchNode targetNode = KeyIntervalAStar::SearchNode(std::nullopt, g, g,
                                                                                   current.keyInterval,
                                                                                   std::nullopt,
                                                                                   std::nullopt, std::nullopt,
                                                                                   finalWaypoints, true);
            return targetNode;
        }
        else
        {
            //logger::log_info("no up/down vertex found before transition vertex for target");
            finalWaypoints.push_back(transitionVertex);
            finalWaypoints.push_back(target);

            double g = 0.0;
            for (size_t i = 1; i < finalWaypoints.size(); ++i)
            {
                g += heuristic(finalWaypoints[i - 1], finalWaypoints[i]);
            }

            KeyIntervalAStar::SearchNode targetNode = KeyIntervalAStar::SearchNode(std::nullopt, g, g,
                                                                                   current.keyInterval,
                                                                                   std::nullopt,
                                                                                   std::nullopt, std::nullopt,
                                                                                   finalWaypoints, true);
            return targetNode;
        }

    }

    // check if there are key points between last waypoint and target vertex
    checkAndAddUpDownVertex(target, current.waypoints.back(), finalWaypoints, current);

    finalWaypoints.push_back(target);

    double g = 0.0;
    for (size_t i = 1; i < finalWaypoints.size(); ++i)
    {
        g += heuristic(finalWaypoints[i - 1], finalWaypoints[i]);
    }

    //logger::log_info("final waypoints: " + //logger::vectorToString(finalWaypoints));

    KeyIntervalAStar::SearchNode targetNode(std::nullopt, g, g, current.keyInterval, std::nullopt,
                                            std::nullopt, std::nullopt, finalWaypoints, true);
    //logger::log_info("push target node");
    return targetNode;
}

std::pair<double, double> KeyIntervalAStar::calcGAndH(const std::vector<Vertex> &waypoints,
                                                      const IntervalKey &evaluationInterval,
                                                      const Vertex &target) const
{
    double gValue = 0.0;
    double hValue = 0.0;

    //logger::log_info("calculate g and h for evaluation interval: " + std::to_string(evaluationInterval.y) + "," + std::to_string(evaluationInterval.start) + "," + std::to_string(evaluationInterval.end));
    //logger::log_info("waypoints: " + //logger::vectorToString(waypoints));

    // calculate cost between adjacent points in waypoints
    for (size_t i = 1; i < waypoints.size(); ++i)
    {
        gValue += heuristic(waypoints[i - 1], waypoints[i]);
        //logger::log_info("gValue += heuristic(" + std::to_string(waypoints[i - 1].x) + "," + std::to_string(waypoints[i - 1].y) + "," + std::to_string(waypoints[i].x) + "," + std::to_string(waypoints[i].y) + "): " + std::to_string(gValue));
    }

    // calculate cost of last segment
    if (!preprocess_->isVertexInKeyInterval(waypoints.back(), evaluationInterval))
    {
        // otherwise calculate cost based on target vertex position
        if (target.x <= evaluationInterval.start)
        {
            Vertex evaluationVertex = Vertex(evaluationInterval.start, evaluationInterval.y);
            //logger::log_info("evaluation vertex: " + std::to_string(evaluationVertex.x) + "," + std::to_string(evaluationVertex.y));
            // if target x is less than or equal to interval start
            gValue += heuristic(waypoints.back(), evaluationVertex);
            hValue = heuristic(evaluationVertex, target);
        }
        else if (target.x >= evaluationInterval.end)
        {
            Vertex evaluationVertex = Vertex(evaluationInterval.end, evaluationInterval.y);
            //logger::log_info("evaluation vertex: " + std::to_string(evaluationVertex.x) + "," + std::to_string(evaluationVertex.y));
            // if target x is greater than or equal to interval end
            gValue += heuristic(waypoints.back(), evaluationVertex);
            hValue = heuristic(evaluationVertex, target);
        }
        else
        {
            Vertex evaluationVertex = Vertex(target.x, evaluationInterval.y);
            //logger::log_info("evaluation vertex: " + std::to_string(evaluationVertex.x) + "," + std::to_string(evaluationVertex.y));
            // if target x is in interval
            gValue += heuristic(waypoints.back(), evaluationVertex);
            hValue = heuristic(evaluationVertex, target);
        }
    }
    else
    {
        //logger::log_info("calculate g and h with last waypoint in current key interval: " + std::to_string(waypoints.back().x) + "," + std::to_string(waypoints.back().y));
        hValue = heuristic(waypoints.back(), target);
    }

    //logger::log_info("gValue: " + std::to_string(gValue) + ", hValue: " + std::to_string(hValue));

    return std::make_pair(gValue, hValue);
}

std::vector<Vertex> KeyIntervalAStar::constructPath(std::vector<Vertex> waypoints) const
{
    //logger::log_info("construct path with waypoints: " + //logger::vectorToString(waypoints));
    // use waypoints to reconstruct full path
    std::vector<Vertex> path;
    if (waypoints.empty())
        return path;
    const auto &grid = preprocess_->getGrid();
    for (size_t i = 0; i < waypoints.size() - 1; ++i)
    {
        // call A* algorithm to get path between two waypoints, remove first point to avoid duplicate
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
            // if A* failed
            //logger::log_error("A* failed with waypoints: " + //logger::vectorToString(waypoints));
            return {};
        }
    }
    return path;
}

void KeyIntervalAStar::insertNode(const SearchNode &node,
                                  std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> &openList,
                                  std::unordered_map<IntervalKey, double, IntervalKeyHash> &gScore)
{
    // for target node, directly insert into openList, no g value comparison
    if (node.isTarget)
    {
        openList.push(node);
        return;
    }

    // check if g value is better
    if (gScore.find(node.keyInterval.value()) == gScore.end() || node.g < gScore[node.keyInterval.value()])
    {
        gScore[node.keyInterval.value()] = node.g;
        openList.push(node);
        //logger::log_info("push new node: " + std::to_string(node.keyInterval.value().y) + "," + std::to_string(node.keyInterval.value().start) + "," + std::to_string(node.keyInterval.value().end));
    }
}

bool KeyIntervalAStar::checkAndAddUpDownVertex(const Vertex &targetVertex,
                                               const Vertex &lastWaypoint,
                                               std::vector<Vertex> &waypoints,
                                               const SearchNode &current)
{
    if (current.downVertex &&
        targetVertex.x < current.downVertex.value().x &&
        lastWaypoint.x < current.downVertex.value().x)
    {
        waypoints.push_back(current.downVertex.value());
        //logger::log_info("add down vertex: " + std::to_string(current.downVertex.value().x) + "," + std::to_string(current.downVertex.value().y));
        return true;
    }

    if (current.upVertex &&
        targetVertex.x > current.upVertex.value().x &&
        lastWaypoint.x > current.upVertex.value().x)
    {
        waypoints.push_back(current.upVertex.value());
        //logger::log_info("add up vertex: " + std::to_string(current.upVertex.value().x) + "," + std::to_string(current.upVertex.value().y));
        return true;
    }
    return false;
}
