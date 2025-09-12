#include "map3d.h"
#include <iostream>

namespace DronePathfinding {

    Map3D::Map3D() {
        voxelMap_ = std::make_unique<VoxelMap3D>();
        mapBounds_ = Point3D{1000, 1000, 1000};
        mapOrigin_ = Point3D{0, 0, 0};
    }

    bool Map3D::loadFromDatabase(PGconn* conn, const std::string& query) {
        if (!voxelMap_) {
            return false;
        }
        return voxelMap_->loadFromDatabase(conn, query);
    }

    bool Map3D::isObstacle(const Point3D& point) const {
        if (!voxelMap_) {
            return false;
        }
        return voxelMap_->isObstacle(point);
    }

    bool Map3D::isObstacle(int x, int y, int z) const {
        return isObstacle(Point3D{x, y, z});
    }

    std::vector<Point3D> Map3D::getObstaclesInRegion(const Point3D& minPoint, const Point3D& maxPoint) const {
        if (!voxelMap_) {
            return {};
        }
        return voxelMap_->getObstaclesInRegion(minPoint, maxPoint);
    }

    bool Map3D::hasCoordMapping(int z, int x, int y) const {
        std::string key = std::to_string(z) + "_" + std::to_string(x) + "_" + std::to_string(y);
        return coordMapping_.find(key) != coordMapping_.end();
    }

    std::tuple<double, double, double> Map3D::getOriginalCoord(int z, int x, int y) const {
        std::string key = std::to_string(z) + "_" + std::to_string(x) + "_" + std::to_string(y);
        auto it = coordMapping_.find(key);
        if (it != coordMapping_.end()) {
            return it->second;
        }
        return std::make_tuple(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
    }

    void Map3D::addCoordMapping(int z, int x, int y, double origX, double origY, double origZ) {
        std::string key = std::to_string(z) + "_" + std::to_string(x) + "_" + std::to_string(y);
        coordMapping_[key] = std::make_tuple(origX, origY, origZ);
    }

    void Map3D::printStatistics() const {
        if (voxelMap_) {
            voxelMap_->printStatistics();
        }
        std::cout << "Coordinate mappings: " << coordMapping_.size() << std::endl;
    }

    size_t Map3D::getTotalObstacles() const {
        if (voxelMap_) {
            return voxelMap_->getTotalObstacles();
        }
        return 0;
    }

    void Map3D::clear() {
        if (voxelMap_) {
            voxelMap_->clear();
        }
        coordMapping_.clear();
    }

} // namespace DronePathfinding