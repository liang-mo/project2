#pragma once

#include "types.h"
#include <memory>
#include <vector>

namespace DronePathfinding {

    class Node {
    public:
        Point3D point;
        std::shared_ptr<Node> parent;
        double gCost;
        double hCost;
        double safetyCost;
        int timeStep;
        Direction direction;
        int droneId;

        Node(const Point3D& p, const Point3D& target, int droneId = 0, int timeStep = 0,
             double g = 0.0, Direction dir = Direction::NONE, double safety = 0.0);

        double getFCost() const;
        bool operator>(const Node& other) const;
        bool operator<(const Node& other) const;

        static double calculateHeuristic(const Point3D& from, const Point3D& to);
        std::vector<Point3D> getPath() const;
    };

    using NodePtr = std::shared_ptr<Node>;

} // namespace DronePathfinding