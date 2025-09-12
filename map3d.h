#pragma once

#include "types.h"
#include "VoxelMap3D.h"
#include <memory>
#include <unordered_map>
#include <tuple>

namespace DronePathfinding {

    class Map3D {
    private:
        std::unique_ptr<VoxelMap3D> voxelMap_;
        std::unordered_map<std::string, std::tuple<double, double, double>> coordMapping_;
        Point3D mapBounds_;
        Point3D mapOrigin_;

    public:
        Map3D();
        ~Map3D() = default;

        // 从数据库加载
        bool loadFromDatabase(PGconn* conn, const std::string& query);

        // 障碍物检测
        bool isObstacle(const Point3D& point) const;
        bool isObstacle(int x, int y, int z) const;

        // 获取障碍物
        std::vector<Point3D> getObstaclesInRegion(const Point3D& minPoint, const Point3D& maxPoint) const;

        // 坐标映射
        bool hasCoordMapping(int z, int x, int y) const;
        std::tuple<double, double, double> getOriginalCoord(int z, int x, int y) const;
        void addCoordMapping(int z, int x, int y, double origX, double origY, double origZ);

        // 地图边界
        Point3D getMapBounds() const { return mapBounds_; }
        Point3D getMapOrigin() const { return mapOrigin_; }
        void setMapBounds(const Point3D& bounds) { mapBounds_ = bounds; }
        void setMapOrigin(const Point3D& origin) { mapOrigin_ = origin; }

        // 统计信息
        void printStatistics() const;
        size_t getTotalObstacles() const;

        // 清理
        void clear();
    };

} // namespace DronePathfinding