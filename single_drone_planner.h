#pragma once

#include "types.h"
#include "map3d.h"
#include "node.h"
#include <unordered_set>
#include <queue>
#include <memory>

namespace DronePathfinding {

    class SingleDronePlanner {
    private:
        const Map3D& map_;
        std::unordered_set<SpaceTimePoint, SpaceTimePointHash> reservations_;
        
        // 统计信息
        int nodesExplored_;
        double planningTime_;
        std::string lastErrorMessage_;

    public:
        explicit SingleDronePlanner(const Map3D& map);
        ~SingleDronePlanner() = default;

        // 路径规划
        Path planPath(const DroneInfo& drone);
        Path planPathWithConstraints(const DroneInfo& drone, 
                                   const std::vector<Constraint>& constraints);

        // 预约系统
        void setReservations(const std::unordered_set<SpaceTimePoint, SpaceTimePointHash>& reservations);
        void clearReservations();
        void addReservation(const SpaceTimePoint& reservation);

        // 统计信息
        int getNodesExplored() const { return nodesExplored_; }
        double getPlanningTime() const { return planningTime_; }
        std::string getLastError() const { return lastErrorMessage_; }

    private:
        // A*算法实现
        Path aStarSearch(const Point3D& start, const Point3D& goal, int droneId);
        Path aStarSearchWithConstraints(const Point3D& start, const Point3D& goal, int droneId,
                                      const std::vector<Constraint>& constraints);

        // 辅助函数
        std::vector<NodePtr> getNeighbors(const NodePtr& node, const Point3D& goal);
        bool isValidMove(const Point3D& from, const Point3D& to, int timeStep, int droneId) const;
        bool isReserved(const Point3D& point, int timeStep) const;
        bool violatesConstraints(const Point3D& point, int timeStep, int droneId,
                               const std::vector<Constraint>& constraints) const;
        double calculateSafetyCost(const Point3D& point, int timeStep) const;
        Path reconstructPath(const NodePtr& goalNode) const;
    };

} // namespace DronePathfinding