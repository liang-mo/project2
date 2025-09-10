#pragma once
#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include "types.h"
#include <libpq-fe.h> 

namespace DronePathfinding {

    // Structure to represent a hierarchical grid cell
    struct GridCell {
        std::string code7;  // 7-level Beidou code (e.g., N51H1342861037)
        std::vector<std::string> code8List;  // List of 8-level codes
        // Note: All code8 entries are obstacles since only obstacle grids are stored

        GridCell() = default;
        GridCell(const std::string& code7) : code7(code7) {}
    };

    // Beidou code utilities
    class BeidouCodeUtils {
    public:
        // Convert Beidou code to longitude/latitude coordinates
        static Point3D beidouToCoordinates(const std::string& beidouCode);

        // Convert Beidou code to longitude/latitude (double precision)
        static std::pair<double, double> beidouToLonLat(const std::string& beidouCode);

        // Convert coordinates to Beidou code
        static std::string coordinatesToBeidou(const Point3D& point);

        // Extract parent code (7-level from 8-level)
        static std::string getParentCode(const std::string& code8);

        // Check if a Beidou code is valid
        static bool isValidBeidouCode(const std::string& code);

        // Get grid bounds for a Beidou code (in lon/lat)
        static std::pair<std::pair<double, double>, std::pair<double, double>> getGridBounds(const std::string& beidouCode);

        // Parse Beidou code components
        static bool parseBeidouCode(const std::string& code, int& level, std::string& gridInfo);
    };

    // Grid-based map for hierarchical obstacle storage
    class CGrid4Map {
    private:
        std::unordered_map<std::string, GridCell> gridCells;  // code7 -> GridCell
        std::unordered_set<std::string> obstacleCode8Set;     // Fast lookup for obstacle code8s

        // Grid resolution parameters
        double gridResolution;
        Point3D mapOrigin;
        Point3D mapBounds;

    public:
        CGrid4Map(double resolution = 1.0);

        // Load grid data from database
        void loadFromDatabase(PGconn* conn, const std::string& query);

        // Add a grid cell with its obstacle data
        void addGridCell(const std::string& code7,
            const std::vector<std::string>& code8List);

        // Check if a specific coordinate is an obstacle
        bool isObstacle(const Point3D& point) const;

        // Check if a specific 8-level Beidou code is an obstacle
        bool isObstacleByCode8(const std::string& code8) const;

        // Get all obstacle coordinates in a region
        std::vector<Point3D> getObstaclesInRegion(const Point3D& minPoint,
            const Point3D& maxPoint) const;

        // Get all obstacle coordinates with longitude/latitude
        std::vector<std::pair<Point3D, std::pair<double, double>>> getObstaclesWithLonLat() const;

        // Get grid cell by 7-level code
        const GridCell* getGridCell(const std::string& code7) const;

        // Clear all grid data
        void clear();

        // Get statistics
        size_t getGridCellCount() const { return gridCells.size(); }
        size_t getObstacleCount() const { return obstacleCode8Set.size(); }

        // Utility methods
        std::vector<Point3D> getAllObstacleCoordinates() const;
        std::vector<std::string> getObstacleCode8List() const;
    };

    
} // namespace DronePathfinding