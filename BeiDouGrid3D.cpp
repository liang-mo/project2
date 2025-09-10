#include "BeiDouGrid3D.h"
#include "types.h"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace Grid {

    std::string BeiDouGrid3D::coordinatesToBeidou(const DronePathfinding::Point3D& point) const {
        // Convert point coordinates to longitude/latitude
        // This is a simplified implementation - you'll need to adapt based on your coordinate system
        auto lonLat = pointToLonLat(point);
        return calculateBeidouCode(lonLat.first, lonLat.second, 9); // 9-level code
    }

    DronePathfinding::Point3D BeiDouGrid3D::beidouToCoordinates(const std::string& beidouCode) const {
        if (!isValidBeidouCode(beidouCode)) {
            return DronePathfinding::Point3D{0, 0, 0};
        }
        
        auto center = calculateCenterFromCode(beidouCode);
        return lonLatToPoint(center.first, center.second, 0);
    }

    std::string BeiDouGrid3D::expandToFullCode(const std::string& code6, int subGridIndex) const {
        if (!isValidBeidouCode(code6) || subGridIndex < 0 || subGridIndex >= 512) {
            return "";
        }
        
        // Convert sub-grid index to 3-level extension
        // 512 = 8^3, so we need 3 more levels (8x8x8 subdivision)
        std::string extension = "";
        int remaining = subGridIndex;
        
        for (int level = 0; level < 3; level++) {
            int digit = remaining % 8;
            extension = std::to_string(digit) + extension;
            remaining /= 8;
        }
        
        return code6 + extension;
    }

    DronePathfinding::Point3D BeiDouGrid3D::subGridToCoordinates(const std::string& code6, int subGridIndex) const {
        std::string fullCode = expandToFullCode(code6, subGridIndex);
        return beidouToCoordinates(fullCode);
    }

    std::pair<std::pair<double, double>, std::pair<double, double>> 
    BeiDouGrid3D::getGridBounds(const std::string& beidouCode) const {
        if (!isValidBeidouCode(beidouCode)) {
            return {{0, 0}, {0, 0}};
        }
        
        auto center = calculateCenterFromCode(beidouCode);
        double gridSize = getGridSizeAtLevel(beidouCode.length() - 1); // Subtract 1 for the 'N' prefix
        
        double halfSize = gridSize / 2.0;
        double minLon = center.first - halfSize;
        double maxLon = center.first + halfSize;
        double minLat = center.second - halfSize;
        double maxLat = center.second + halfSize;
        
        return {{minLon, minLat}, {maxLon, maxLat}};
    }

    bool BeiDouGrid3D::isValidBeidouCode(const std::string& code) const {
        if (code.empty() || code[0] != 'N') {
            return false;
        }
        
        // Check if all characters after 'N' are digits
        for (size_t i = 1; i < code.length(); i++) {
            if (!std::isdigit(code[i])) {
                return false;
            }
        }
        
        // Check valid length (typically 6-12 levels)
        return code.length() >= 7 && code.length() <= 13;
    }

    std::string BeiDouGrid3D::getParentCode(const std::string& code) const {
        if (code.length() <= 2) { // 'N' + at least one digit
            return "";
        }
        return code.substr(0, code.length() - 1);
    }

    std::pair<double, double> BeiDouGrid3D::pointToLonLat(const DronePathfinding::Point3D& point) const {
        // This is a simplified conversion - adapt based on your coordinate system
        // Assuming point coordinates are in some projected system that needs conversion
        double lon = static_cast<double>(point.x) / 100000.0; // Example scaling
        double lat = static_cast<double>(point.y) / 100000.0; // Example scaling
        return {lon, lat};
    }

    DronePathfinding::Point3D BeiDouGrid3D::lonLatToPoint(double lon, double lat, double height) const {
        // Reverse conversion from longitude/latitude to point coordinates
        int x = static_cast<int>(lon * 100000.0); // Example scaling
        int y = static_cast<int>(lat * 100000.0); // Example scaling
        int z = static_cast<int>(height);
        return DronePathfinding::Point3D{x, y, z};
    }

    std::string BeiDouGrid3D::calculateBeidouCode(double lon, double lat, int level) const {
        // Simplified Beidou grid calculation
        // This is a basic implementation - you'll need to implement the actual Beidou grid algorithm
        
        std::string code = "N";
        
        // Normalize coordinates to [0, 1] range
        double normLon = (lon + 180.0) / 360.0; // Longitude: -180 to 180 -> 0 to 1
        double normLat = (lat + 90.0) / 180.0;  // Latitude: -90 to 90 -> 0 to 1
        
        // Generate code digits based on recursive subdivision
        for (int i = 0; i < level; i++) {
            int digit = 0;
            
            // 8-way subdivision (2x2x2 for 3D, but we'll use 2x4 for 2D)
            if (normLon >= 0.5) {
                digit += 4;
                normLon = (normLon - 0.5) * 2.0;
            } else {
                normLon = normLon * 2.0;
            }
            
            if (normLat >= 0.5) {
                digit += 2;
                normLat = (normLat - 0.5) * 2.0;
            } else {
                normLat = normLat * 2.0;
            }
            
            // Add height subdivision if needed (simplified)
            // digit += heightComponent;
            
            code += std::to_string(digit % 8);
        }
        
        return code;
    }

    std::pair<double, double> BeiDouGrid3D::calculateCenterFromCode(const std::string& code) const {
        if (code.empty() || code[0] != 'N') {
            return {0.0, 0.0};
        }
        
        double lon = -180.0; // Start from western edge
        double lat = -90.0;  // Start from southern edge
        double lonSize = 360.0; // Initial longitude range
        double latSize = 180.0; // Initial latitude range
        
        // Process each digit
        for (size_t i = 1; i < code.length(); i++) {
            int digit = code[i] - '0';
            
            lonSize /= 2.0;
            latSize /= 2.0;
            
            // Decode the digit to determine subdivision
            if (digit >= 4) {
                lon += lonSize;
                digit -= 4;
            }
            
            if (digit >= 2) {
                lat += latSize;
                digit -= 2;
            }
            
            // Handle remaining bits for height if needed
        }
        
        // Return center of the cell
        return {lon + lonSize / 2.0, lat + latSize / 2.0};
    }

    double BeiDouGrid3D::getGridSizeAtLevel(int level) const {
        // Calculate grid size at given level
        // Each level subdivides by factor of 2 in each dimension
        double baseSize = 360.0; // Degrees (longitude range)
        return baseSize / std::pow(2.0, level);
    }

} // namespace Grid