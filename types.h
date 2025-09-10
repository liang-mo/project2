#pragma once

#include <vector>
#include <memory>
#include <unordered_set>
#include <functional>
#include <nlohmann/json.hpp>

namespace DronePathfinding {

    // Basic 3D point structure
    struct Point3D {
        int x, y, z;
        
        Point3D() : x(0), y(0), z(0) {}
        Point3D(int x_, int y_, int z_) : x(x_), y(y_), z(z_) {}
        
        bool operator==(const Point3D& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
        
        bool operator!=(const Point3D& other) const {
            return !(*this == other);
        }
        
        Point3D operator+(const Point3D& other) const {
            return Point3D(x + other.x, y + other.y, z + other.z);
        }
        
        Point3D operator-(const Point3D& other) const {
            return Point3D(x - other.x, y - other.y, z - other.z);
        }
    };

    // Hash function for Point3D
    struct Point3DHash {
        std::size_t operator()(const Point3D& p) const {
            return std::hash<int>()(p.x) ^ 
                   (std::hash<int>()(p.y) << 1) ^ 
                   (std::hash<int>()(p.z) << 2);
        }
    };

    // Space-time point for conflict detection
    struct SpaceTimePoint {
        Point3D point;
        int timeStep;
        
        SpaceTimePoint(const Point3D& p, int t) : point(p), timeStep(t) {}
        
        bool operator==(const SpaceTimePoint& other) const {
            return point == other.point && timeStep == other.timeStep;
        }
    };

    // Hash function for SpaceTimePoint
    struct SpaceTimePointHash {
        std::size_t operator()(const SpaceTimePoint& stp) const {
            return Point3DHash()(stp.point) ^ (std::hash<int>()(stp.timeStep) << 3);
        }
    };

    // Movement directions in 3D space
    enum class Direction {
        NONE = 0,
        NORTH, SOUTH, EAST, WEST, UP, DOWN,
        // Diagonal movements
        NORTH_EAST, NORTH_WEST, SOUTH_EAST, SOUTH_WEST,
        // 3D diagonal movements
        NORTH_UP, NORTH_DOWN, SOUTH_UP, SOUTH_DOWN,
        EAST_UP, EAST_DOWN, WEST_UP, WEST_DOWN,
        // Full 3D diagonal
        NORTH_EAST_UP, NORTH_EAST_DOWN, NORTH_WEST_UP, NORTH_WEST_DOWN,
        SOUTH_EAST_UP, SOUTH_EAST_DOWN, SOUTH_WEST_UP, SOUTH_WEST_DOWN
    };

    // Drone information
    struct DroneInfo {
        int id;
        Point3D start;
        Point3D goal;
        double maxSpeed;
        double safetyRadius;
        int priority;  // Lower number = higher priority
        
        DroneInfo() : id(0), maxSpeed(1.0), safetyRadius(1.0), priority(0) {}
        DroneInfo(int id_, const Point3D& start_, const Point3D& goal_, 
                 double speed = 1.0, double radius = 1.0, int prio = 0)
            : id(id_), start(start_), goal(goal_), maxSpeed(speed), 
              safetyRadius(radius), priority(prio) {}
    };

    // Forward declarations
    class Node;
    using NodePtr = std::shared_ptr<Node>;
    using Path = std::vector<NodePtr>;
    using Paths = std::vector<Path>;

    // Conflict types
    enum class ConflictType {
        VERTEX,     // Two drones at same position at same time
        EDGE,       // Two drones crossing paths
        FOLLOWING   // Two drones too close to each other
    };

    // Conflict information
    struct Conflict {
        ConflictType type;
        int drone1Id;
        int drone2Id;
        int timeStep;
        Point3D location1;
        Point3D location2;
        
        Conflict(ConflictType t, int d1, int d2, int time, 
                const Point3D& loc1, const Point3D& loc2)
            : type(t), drone1Id(d1), drone2Id(d2), timeStep(time), 
              location1(loc1), location2(loc2) {}
    };

    // Constraint for CBS algorithm
    struct Constraint {
        int droneId;
        Point3D location;
        int timeStep;
        ConflictType type;
        
        Constraint(int id, const Point3D& loc, int time, ConflictType t)
            : droneId(id), location(loc), timeStep(time), type(t) {}
        
        bool operator==(const Constraint& other) const {
            return droneId == other.droneId && 
                   location == other.location && 
                   timeStep == other.timeStep && 
                   type == other.type;
        }
    };

    // Hash function for Constraint
    struct ConstraintHash {
        std::size_t operator()(const Constraint& c) const {
            return std::hash<int>()(c.droneId) ^ 
                   Point3DHash()(c.location) ^ 
                   (std::hash<int>()(c.timeStep) << 1) ^
                   (std::hash<int>()(static_cast<int>(c.type)) << 2);
        }
    };

    // Planning result
    struct PlanningResult {
        bool success;
        Paths paths;
        std::vector<Conflict> conflicts;
        double planningTime;
        int iterationCount;
        int nodesExplored;
        std::string errorMessage;
        
        PlanningResult() : success(false), planningTime(0.0), 
                          iterationCount(0), nodesExplored(0) {}
    };

    // Movement cost calculation
    inline double calculateMovementCost(const Point3D& from, const Point3D& to) {
        int dx = std::abs(to.x - from.x);
        int dy = std::abs(to.y - from.y);
        int dz = std::abs(to.z - from.z);
        
        // Euclidean distance
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    // Get all possible movement directions
    inline std::vector<Point3D> getMovementDirections(bool allow3D = true, bool allowDiagonal = true) {
        std::vector<Point3D> directions;
        
        // Basic 6-directional movement
        directions.push_back(Point3D(1, 0, 0));   // East
        directions.push_back(Point3D(-1, 0, 0));  // West
        directions.push_back(Point3D(0, 1, 0));   // North
        directions.push_back(Point3D(0, -1, 0));  // South
        
        if (allow3D) {
            directions.push_back(Point3D(0, 0, 1));   // Up
            directions.push_back(Point3D(0, 0, -1));  // Down
        }
        
        if (allowDiagonal) {
            // 2D diagonal movements
            directions.push_back(Point3D(1, 1, 0));   // North-East
            directions.push_back(Point3D(1, -1, 0));  // South-East
            directions.push_back(Point3D(-1, 1, 0));  // North-West
            directions.push_back(Point3D(-1, -1, 0)); // South-West
            
            if (allow3D) {
                // 3D diagonal movements
                directions.push_back(Point3D(1, 0, 1));   // East-Up
                directions.push_back(Point3D(1, 0, -1));  // East-Down
                directions.push_back(Point3D(-1, 0, 1));  // West-Up
                directions.push_back(Point3D(-1, 0, -1)); // West-Down
                directions.push_back(Point3D(0, 1, 1));   // North-Up
                directions.push_back(Point3D(0, 1, -1));  // North-Down
                directions.push_back(Point3D(0, -1, 1));  // South-Up
                directions.push_back(Point3D(0, -1, -1)); // South-Down
                
                // Full 3D diagonal movements
                directions.push_back(Point3D(1, 1, 1));   // North-East-Up
                directions.push_back(Point3D(1, 1, -1));  // North-East-Down
                directions.push_back(Point3D(1, -1, 1));  // South-East-Up
                directions.push_back(Point3D(1, -1, -1)); // South-East-Down
                directions.push_back(Point3D(-1, 1, 1));  // North-West-Up
                directions.push_back(Point3D(-1, 1, -1)); // North-West-Down
                directions.push_back(Point3D(-1, -1, 1)); // South-West-Up
                directions.push_back(Point3D(-1, -1, -1));// South-West-Down
            }
        }
        
        return directions;
    }

    // Utility function to convert paths to JSON
    inline nlohmann::json pathsToJson(const Paths& paths) {
        nlohmann::json result = nlohmann::json::array();
        
        for (size_t i = 0; i < paths.size(); i++) {
            nlohmann::json pathJson;
            pathJson["drone_id"] = static_cast<int>(i);
            pathJson["points"] = nlohmann::json::array();
            
            for (const auto& node : paths[i]) {
                nlohmann::json pointJson;
                pointJson["x"] = node->point.x;
                pointJson["y"] = node->point.y;
                pointJson["z"] = node->point.z;
                pointJson["time_step"] = node->timeStep;
                pathJson["points"].push_back(pointJson);
            }
            
            result.push_back(pathJson);
        }
        
        return result;
    }

} // namespace DronePathfinding