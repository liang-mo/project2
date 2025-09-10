https://stackblitz.com/storage/blobs/eyJfcmFpbHMiOnsibWVzc2FnZSI6IkJBaHBCSHhyNHdFPSIsImV4cCI6bnVsbCwicHVyIjoiYmxvYl9pZCJ9fQ==--21bc48ef7b6f0b479dc3c1da873b41b31658825b//BeiDouGrid3D.h

#include "PositioningGrid.h"
#include <memory>
#include <string>
#include <utility>
#include <vector>

// Forward declaration for DronePathfinding types
namespace DronePathfinding {
    struct Point3D;
}

namespace Grid
{
    class BeiDouGrid3D : public PositioningGrid
    {
    public:
        BeiDouGrid3D() = default;
        virtual ~BeiDouGrid3D() = default;

        // Convert point to Beidou code
        std::string coordinatesToBeidou(const DronePathfinding::Point3D& point) const;
        
        // Convert Beidou code to point
        DronePathfinding::Point3D beidouToCoordinates(const std::string& beidouCode) const;
        
        // Expand 6-level code to full 9-level code using sub-grid index
        std::string expandToFullCode(const std::string& code6, int subGridIndex) const;
        
        // Convert sub-grid to coordinates
        DronePathfinding::Point3D subGridToCoordinates(const std::string& code6, int subGridIndex) const;
        
        // Convert sub-grid to geographic coordinates (longitude, latitude, height)
        DronePathfinding::Point3D subGridToGeoCoordinates(const std::string& code6, int subGridIndex) const;
        
        // Get grid bounds for a Beidou code (returns min/max lon/lat)
        std::pair<std::pair<double, double>, std::pair<double, double>> getGridBounds(const std::string& beidouCode) const;
        
        // Validate Beidou code format
        bool isValidBeidouCode(const std::string& code) const;
        
        // Extract parent code (6-level from higher level)
        std::string getParentCode(const std::string& code) const;
        
        // Convert between coordinate systems
        std::pair<double, double> pointToLonLat(const DronePathfinding::Point3D& point) const;
        DronePathfinding::Point3D lonLatToPoint(double lon, double lat, double height = 0) const;
        
    private:
        // Internal helper methods for Beidou grid calculations
        std::string calculateBeidouCode(double lon, double lat, int level) const;
        std::pair<double, double> calculateCenterFromCode(const std::string& code) const;
        double getGridSizeAtLevel(int level) const;
    };

    static std::unique_ptr<PositioningGrid> GetPositioningGridByType(const GridType& type)
    {
        std::unique_ptr<PositioningGrid> ptr = nullptr;