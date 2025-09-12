#include "single_drone_planner.h"
#include "config.h"
#include <algorithm>
#include <chrono>
#include <iostream>

namespace DronePathfinding {

    SingleDronePlanner::SingleDronePlanner(const Map3D& map) 
        : map_(map), nodesExplored_(0), planningTime_(0.0) {
    }

    Path SingleDronePlanner::planPath(const DroneInfo& drone) {
        std::vector<Constraint> emptyConstraints;
        return planPathWithConstraints(drone, emptyConstraints);
    }

    Path SingleDronePlanner::planPathWithConstraints(const DroneInfo& drone, 
                                                    const std::vector<Constraint>& constraints) {
        auto startTime = std::chrono::high_resolution_clock::now();
        nodesExplored_ = 0;
        lastErrorMessage_.clear();

        Path path = aStarSearchWithConstraints(drone.start, drone.goal, drone.id, constraints);

        auto endTime = std::chrono::high_resolution_clock::now();
        planningTime_ = std::chrono::duration<double>(endTime - startTime).count();

        return path;
    }

    void SingleDronePlanner::setReservations(const std::unordered_set<SpaceTimePoint, SpaceTimePointHash>& reservations) {
        reservations_ = reservations;
    }

    void SingleDronePlanner::clearReservations() {
        reservations_.clear();
    }

    void SingleDronePlanner::addReservation(const SpaceTimePoint& reservation) {
        reservations_.insert(reservation);
    }

    Path SingleDronePlanner::aStarSearch(const Point3D& start, const Point3D& goal, int droneId) {
        std::vector<Constraint> emptyConstraints;
        return aStarSearchWithConstraints(start, goal, droneId, emptyConstraints);
    }

    Path SingleDronePlanner::aStarSearchWithConstraints(const Point3D& start, const Point3D& goal, int droneId,
                                                       const std::vector<Constraint>& constraints) {
        // 优先队列：最小堆
        auto compare = [](const NodePtr& a, const NodePtr& b) {
            return a->getFCost() > b->getFCost();
        };
        std::priority_queue<NodePtr, std::vector<NodePtr>, decltype(compare)> openSet(compare);
        
        std::unordered_set<std::string> closedSet;
        
        // 创建起始节点
        auto startNode = std::make_shared<Node>(start, goal, droneId, 0);
        openSet.push(startNode);
        
        nodesExplored_ = 0;
        
        while (!openSet.empty() && nodesExplored_ < g_config.maxNodesExplored) {
            auto current = openSet.top();
            openSet.pop();
            nodesExplored_++;
            
            // 生成节点的唯一标识
            std::string nodeKey = std::to_string(current->point.x) + "_" + 
                                std::to_string(current->point.y) + "_" + 
                                std::to_string(current->point.z) + "_" + 
                                std::to_string(current->timeStep);
            
            if (closedSet.find(nodeKey) != closedSet.end()) {
                continue;
            }
            closedSet.insert(nodeKey);
            
            // 检查是否到达目标
            if (current->point == goal) {
                return reconstructPath(current);
            }
            
            // 获取邻居节点
            auto neighbors = getNeighbors(current, goal);
            
            for (auto& neighbor : neighbors) {
                std::string neighborKey = std::to_string(neighbor->point.x) + "_" + 
                                        std::to_string(neighbor->point.y) + "_" + 
                                        std::to_string(neighbor->point.z) + "_" + 
                                        std::to_string(neighbor->timeStep);
                
                if (closedSet.find(neighborKey) != closedSet.end()) {
                    continue;
                }
                
                // 检查约束
                if (violatesConstraints(neighbor->point, neighbor->timeStep, droneId, constraints)) {
                    continue;
                }
                
                // 检查预约
                if (isReserved(neighbor->point, neighbor->timeStep)) {
                    continue;
                }
                
                neighbor->parent = current;
                openSet.push(neighbor);
            }
        }
        
        lastErrorMessage_ = "No path found";
        return Path();
    }

    std::vector<NodePtr> SingleDronePlanner::getNeighbors(const NodePtr& node, const Point3D& goal) {
        std::vector<NodePtr> neighbors;
        auto directions = getMovementDirections(true, true);
        
        for (const auto& dir : directions) {
            Point3D newPoint = node->point + dir;
            
            // 检查边界
            if (newPoint.x < 0 || newPoint.y < 0 || newPoint.z < 0) {
                continue;
            }
            
            // 检查是否是障碍物
            if (map_.isObstacle(newPoint)) {
                continue;
            }
            
            // 检查移动是否有效
            if (!isValidMove(node->point, newPoint, node->timeStep + 1, node->droneId)) {
                continue;
            }
            
            // 计算代价
            double moveCost = calculateMovementCost(node->point, newPoint);
            double gCost = node->gCost + moveCost;
            double safetyCost = calculateSafetyCost(newPoint, node->timeStep + 1);
            
            auto neighbor = std::make_shared<Node>(newPoint, goal, node->droneId, 
                                                 node->timeStep + 1, gCost, Direction::NONE, safetyCost);
            neighbors.push_back(neighbor);
        }
        
        // 添加等待动作（停留在原地）
        if (!isReserved(node->point, node->timeStep + 1)) {
            double safetyCost = calculateSafetyCost(node->point, node->timeStep + 1);
            auto waitNode = std::make_shared<Node>(node->point, goal, node->droneId, 
                                                 node->timeStep + 1, node->gCost + 1.0, Direction::NONE, safetyCost);
            neighbors.push_back(waitNode);
        }
        
        return neighbors;
    }

    bool SingleDronePlanner::isValidMove(const Point3D& from, const Point3D& to, int timeStep, int droneId) const {
        // 检查移动距离是否合理
        double distance = calculateMovementCost(from, to);
        if (distance > 2.0) { // 最大移动距离限制
            return false;
        }
        
        return true;
    }

    bool SingleDronePlanner::isReserved(const Point3D& point, int timeStep) const {
        SpaceTimePoint stp(point, timeStep);
        return reservations_.find(stp) != reservations_.end();
    }

    bool SingleDronePlanner::violatesConstraints(const Point3D& point, int timeStep, int droneId,
                                                const std::vector<Constraint>& constraints) const {
        for (const auto& constraint : constraints) {
            if (constraint.droneId == droneId && 
                constraint.location == point && 
                constraint.timeStep == timeStep) {
                return true;
            }
        }
        return false;
    }

    double SingleDronePlanner::calculateSafetyCost(const Point3D& point, int timeStep) const {
        // 简单的安全代价计算
        double cost = 0.0;
        
        // 检查周围是否有预约
        auto directions = getMovementDirections(true, false);
        for (const auto& dir : directions) {
            Point3D checkPoint = point + dir;
            if (isReserved(checkPoint, timeStep)) {
                cost += g_config.weightSafety;
            }
        }
        
        return cost;
    }

    Path SingleDronePlanner::reconstructPath(const NodePtr& goalNode) const {
        Path path;
        NodePtr current = goalNode;
        
        while (current != nullptr) {
            path.push_back(current);
            current = current->parent;
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }

} // namespace DronePathfinding