#pragma once

#include "types.h"
#include "node.h"
#include <vector>

namespace DronePathfinding {

    class ConflictDetector {
    public:
        // 检测所有路径之间的冲突
        static std::vector<Conflict> detectConflicts(const Paths& paths);
        
        // 检测两条路径之间的冲突
        static std::vector<Conflict> detectConflictsBetweenPaths(const Path& path1, const Path& path2);
        
        // 检测特定类型的冲突
        static std::vector<Conflict> detectVertexConflicts(const Paths& paths);
        static std::vector<Conflict> detectEdgeConflicts(const Paths& paths);
        static std::vector<Conflict> detectFollowingConflicts(const Paths& paths);
        
    private:
        // 冲突检测辅助函数
        static bool isVertexConflict(const NodePtr& node1, const NodePtr& node2);
        static bool isEdgeConflict(const NodePtr& node1a, const NodePtr& node1b,
                                 const NodePtr& node2a, const NodePtr& node2b);
        static bool isFollowingConflict(const NodePtr& node1, const NodePtr& node2, double minDistance);
        
        // 工具函数
        static double calculateDistance(const Point3D& p1, const Point3D& p2);
    };

} // namespace DronePathfinding