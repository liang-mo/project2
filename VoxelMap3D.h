#pragma once

#include "types.h"
#include "BeiDouGrid3D.h"
#include <libpq-fe.h>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <bitset>
#include <string>
#include <iostream>
#include <nlohmann/json.hpp>

namespace DronePathfinding {

    // Bitwise state array for 512 states (64 bytes)
    using StateArray = std::bitset<512>;

    // Hierarchical grid cell structure
    struct HierarchicalGridCell {
        std::string beidouCode6;  // 6-level Beidou code
        StateArray states;        // 512 states as bitset
        
        HierarchicalGridCell() = default;
        HierarchicalGridCell(const std::string& code, const StateArray& stateArray) 
            : beidouCode6(code), states(stateArray) {}
        
        // Check if a specific sub-grid is obstacle (bit position)
        bool isObstacle(int subGridIndex) const {
            return subGridIndex >= 0 && subGridIndex < 512 && states[subGridIndex];
        }
        
        // Set obstacle state for a sub-grid
        void setObstacle(int subGridIndex, bool isObstacle = true) {
            if (subGridIndex >= 0 && subGridIndex < 512) {
                states[subGridIndex] = isObstacle;
            }
        }
        
        // Get obstacle count in this cell
        size_t getObstacleCount() const {
            return states.count();
        }
        
        // Check if cell has any obstacles
        bool hasObstacles() const {
            return states.any();
        }
    };

    class VoxelMap3D {
    private:
        // Hierarchical storage: 6-level code -> grid cell with 512 states
        std::unordered_map<std::string, HierarchicalGridCell> hierarchicalGrid_;
        
        // Fast lookup for obstacle checking
        std::unordered_set<std::string> obstacleVoxels_;  // Full 9-level codes for obstacles
        
        // Grid parameters
        double voxelSize_;
        Point3D mapOrigin_;
        Point3D mapBounds_;
        
        // Coordinate mapping for original coordinates
        std::unordered_map<std::string, std::tuple<double, double, double>> coordMapping_;
        
        // Statistics
        size_t totalCells_;
        size_t totalObstacles_;
        
        // Beidou grid integration
        std::unique_ptr<Grid::BeiDouGrid3D> beidouGrid_;

    public:
        VoxelMap3D(double voxelSize = 1.0) 
            : voxelSize_(voxelSize), totalCells_(0), totalObstacles_(0) {
            beidouGrid_ = std::make_unique<Grid::BeiDouGrid3D>();
        }

        // Static factory method to build from PostgreSQL
        static std::unique_ptr<VoxelMap3D> buildFromPostgres(PGconn* conn, const std::string& query) {
            auto voxelMap = std::make_unique<VoxelMap3D>();
            
            if (!voxelMap->loadFromDatabase(conn, query)) {
                return nullptr;
            }
            
            return voxelMap;
        }

        // Load hierarchical grid data from database
        bool loadFromDatabase(PGconn* conn, const std::string& query) {
            PGresult* res = PQexec(conn, query.c_str());
            
            if (PQresultStatus(res) != PGRES_TUPLES_OK) {
                std::cerr << "Query failed: " << PQerrorMessage(conn) << std::endl;
                PQclear(res);
                return false;
            }

            int nrows = PQntuples(res);
            std::cout << "Loading " << nrows << " hierarchical grid cells..." << std::endl;

            for (int i = 0; i < nrows; i++) {
                // Assuming columns: id, states (bytea), code3d (6-level Beidou code)
                std::string beidouCode6 = PQgetvalue(res, i, 2);  // code3d column
                
                // Parse states array (assuming it's stored as bytea - 64 bytes for 512 bits)
                const char* statesData = PQgetvalue(res, i, 1);
                int statesLength = PQgetlength(res, i, 1);
                
                StateArray states;
                if (statesLength >= 64) {  // 512 bits = 64 bytes
                    // Convert bytea to bitset
                    const unsigned char* bytes = reinterpret_cast<const unsigned char*>(statesData);
                    for (int byteIdx = 0; byteIdx < 64; byteIdx++) {
                        for (int bitIdx = 0; bitIdx < 8; bitIdx++) {
                            int globalBitIdx = byteIdx * 8 + bitIdx;
                            if (globalBitIdx < 512) {
                                states[globalBitIdx] = (bytes[byteIdx] >> bitIdx) & 1;
                            }
                        }
                    }
                }

                // Add to hierarchical grid
                hierarchicalGrid_[beidouCode6] = HierarchicalGridCell(beidouCode6, states);
                
                // Generate obstacle voxel codes for fast lookup
                generateObstacleVoxels(beidouCode6, states);
                
                totalCells_++;
                totalObstacles_ += states.count();
            }

            PQclear(res);
            
            std::cout << "Loaded " << totalCells_ << " cells with " << totalObstacles_ << " obstacles" << std::endl;
            return true;
        }

        // Core obstacle checking method - compatible with existing pathfinding
        bool isObstacle(const Point3D& point) const {
            // Convert point to Beidou code
            std::string fullBeidouCode = pointToBeidouCode(point);
            if (fullBeidouCode.empty()) return false;
            
            // Check in fast lookup set
            return obstacleVoxels_.find(fullBeidouCode) != obstacleVoxels_.end();
        }

        // Alternative obstacle checking by coordinates
        bool isObstacle(int x, int y, int z) const {
            return isObstacle(Point3D{x, y, z});
        }

        // Get obstacles in a region (for pathfinding algorithms)
        std::vector<Point3D> getObstaclesInRegion(const Point3D& minPoint, const Point3D& maxPoint) const {
            std::vector<Point3D> obstacles;
            
            // Iterate through relevant grid cells
            for (const auto& [code6, cell] : hierarchicalGrid_) {
                if (!cell.hasObstacles()) continue;
                
                // Get cell bounds
                auto bounds = beidouGrid_->getGridBounds(code6);
                
                // Check if cell intersects with query region
                if (cellIntersectsRegion(bounds, minPoint, maxPoint)) {
                    // Add obstacle points from this cell
                    addObstaclesFromCell(code6, cell, minPoint, maxPoint, obstacles);
                }
            }
            
            return obstacles;
        }

        // Get obstacles with geographic coordinates
        std::vector<Point3D> getObstaclesGeo(int maxCount = -1) const {
            std::vector<Point3D> obstacles;
            int count = 0;
            
            for (const auto& [code6, cell] : hierarchicalGrid_) {
                if (!cell.hasObstacles()) continue;
                
                for (int subIdx = 0; subIdx < 512; subIdx++) {
                    if (cell.isObstacle(subIdx)) {
                        // Convert to geographic coordinates
                        Point3D geoPoint = subGridToGeoCoordinates(code6, subIdx);
                        obstacles.push_back(geoPoint);
                        
                        if (maxCount > 0 && ++count >= maxCount) {
                            return obstacles;
                        }
                    }
                }
            }
            
            return obstacles;
        }

        // Coordinate mapping methods for compatibility
        bool hasCoordMapping(int z, int x, int y) const {
            std::string key = std::to_string(z) + "_" + std::to_string(x) + "_" + std::to_string(y);
            return coordMapping_.find(key) != coordMapping_.end();
        }

        std::tuple<double, double, double> getOriginalCoord(int z, int x, int y) const {
            std::string key = std::to_string(z) + "_" + std::to_string(x) + "_" + std::to_string(y);
            auto it = coordMapping_.find(key);
            if (it != coordMapping_.end()) {
                return it->second;
            }
            return std::make_tuple(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z));
        }

        // Statistics and utility methods
        void printStatistics() const {
            std::cout << "\n=== VoxelMap3D Statistics ===" << std::endl;
            std::cout << "Total hierarchical cells: " << totalCells_ << std::endl;
            std::cout << "Total obstacles: " << totalObstacles_ << std::endl;
            std::cout << "Voxel size: " << voxelSize_ << std::endl;
            std::cout << "Memory usage (approx): " << getMemoryUsage() << " MB" << std::endl;
            
            // Calculate density
            if (totalCells_ > 0) {
                double density = static_cast<double>(totalObstacles_) / (totalCells_ * 512) * 100.0;
                std::cout << "Obstacle density: " << density << "%" << std::endl;
            }
        }

        size_t getTotalCells() const { return totalCells_; }
        size_t getTotalObstacles() const { return totalObstacles_; }
        double getVoxelSize() const { return voxelSize_; }

        // Get bounds for pathfinding algorithms
        Point3D getMapOrigin() const { return mapOrigin_; }
        Point3D getMapBounds() const { return mapBounds_; }

        // Clear all data
        void clear() {
            hierarchicalGrid_.clear();
            obstacleVoxels_.clear();
            coordMapping_.clear();
            totalCells_ = 0;
            totalObstacles_ = 0;
        }

    private:
        // Generate obstacle voxel codes for fast lookup
        void generateObstacleVoxels(const std::string& code6, const StateArray& states) {
            for (int subIdx = 0; subIdx < 512; subIdx++) {
                if (states[subIdx]) {
                    // Generate full 9-level Beidou code for this sub-grid
                    std::string fullCode = generateFullBeidouCode(code6, subIdx);
                    if (!fullCode.empty()) {
                        obstacleVoxels_.insert(fullCode);
                    }
                }
            }
        }

        // Convert point to Beidou code using BeiDouGrid3D
        std::string pointToBeidouCode(const Point3D& point) const {
            if (beidouGrid_) {
                return beidouGrid_->coordinatesToBeidou(point);
            }
            return "";
        }

        // Generate full 9-level Beidou code from 6-level code and sub-grid index
        std::string generateFullBeidouCode(const std::string& code6, int subIdx) const {
            if (beidouGrid_) {
                return beidouGrid_->expandToFullCode(code6, subIdx);
            }
            return "";
        }

        // Convert sub-grid to geographic coordinates
        Point3D subGridToGeoCoordinates(const std::string& code6, int subIdx) const {
            if (beidouGrid_) {
                return beidouGrid_->subGridToGeoCoordinates(code6, subIdx);
            }
            return Point3D{0, 0, 0};
        }

        // Check if cell intersects with query region
        bool cellIntersectsRegion(const std::pair<std::pair<double, double>, std::pair<double, double>>& bounds,
                                 const Point3D& minPoint, const Point3D& maxPoint) const {
            // Simple bounding box intersection check
            return !(bounds.second.first < minPoint.x || bounds.first.first > maxPoint.x ||
                    bounds.second.second < minPoint.y || bounds.first.second > maxPoint.y);
        }

        // Add obstacles from a cell within the specified region
        void addObstaclesFromCell(const std::string& code6, const HierarchicalGridCell& cell,
                                 const Point3D& minPoint, const Point3D& maxPoint,
                                 std::vector<Point3D>& obstacles) const {
            for (int subIdx = 0; subIdx < 512; subIdx++) {
                if (cell.isObstacle(subIdx)) {
                    Point3D point = subGridToGeoCoordinates(code6, subIdx);
                    
                    // Check if point is within region
                    if (point.x >= minPoint.x && point.x <= maxPoint.x &&
                        point.y >= minPoint.y && point.y <= maxPoint.y &&
                        point.z >= minPoint.z && point.z <= maxPoint.z) {
                        obstacles.push_back(point);
                    }
                }
            }
        }

        // Estimate memory usage in MB
        double getMemoryUsage() const {
            double size = 0.0;
            
            // Hierarchical grid storage
            size += hierarchicalGrid_.size() * (sizeof(std::string) + sizeof(HierarchicalGridCell));
            
            // Obstacle voxels set
            size += obstacleVoxels_.size() * sizeof(std::string);
            
            // Coordinate mapping
            size += coordMapping_.size() * (sizeof(std::string) + sizeof(std::tuple<double, double, double>));
            
            return size / (1024.0 * 1024.0);  // Convert to MB
        }
    };

} // namespace DronePathfinding