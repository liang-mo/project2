#include "multi_drone_planner.h"
#include "config.h"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

namespace DronePathfinding {

    MultiDronePlanner::MultiDronePlanner(const Map3D& map) 
        : map_(map), iterationCount_(0), totalNodesExplored_(0), totalPlanningTime_(0.0) {
        singlePlanner_ = std::make_unique<SingleDronePlanner>(map);
    }

    void MultiDronePlanner::addDrone(const DroneInfo& drone) {
        drones_.push_back(drone);
    }

    void MultiDronePlanner::clearDrones() {
        drones_.clear();
        reset();
    }

    bool MultiDronePlanner::planPaths() {
        auto startTime = std::chrono::high_resolution_clock::now();
        
        reset();
        
        if (drones_.empty()) {
            lastErrorMessage_ = "No drones to plan for";
            return false;
        }
        
        // 按优先级排序无人机
        sortDronesByPriority();
        
        // 使用基于优先级的方法进行规划
        bool success = planWithPriorityBasedApproach();
        
        if (!success) {
            // 如果优先级方法失败，尝试迭代方法
            success = planWithIterativeApproach();
        }
        
        auto endTime = std::chrono::high_resolution_clock::now();
        totalPlanningTime_ = std::chrono::duration<double>(endTime - startTime).count();
        
        // 检测最终冲突
        currentConflicts_ = ConflictDetector::detectConflicts(currentPaths_);
        
        if (g_config.enableDebugOutput) {
            printStatistics();
        }
        
        return success && currentConflicts_.empty();
    }

    void MultiDronePlanner::savePathsToJson(const std::string& filename) const {
        nlohmann::json output = getPathsAsJson();
        
        std::ofstream file(filename);
        if (file.is_open()) {
            file << output.dump(4);
            file.close();
            std::cout << "Paths saved to " << filename << std::endl;
        } else {
            std::cerr << "Failed to save paths to " << filename << std::endl;
        }
    }

    void MultiDronePlanner::printStatistics() const {
        std::cout << "\n=== Multi-Drone Planning Statistics ===" << std::endl;
        std::cout << "Total drones: " << drones_.size() << std::endl;
        std::cout << "Successful paths: " << currentPaths_.size() << std::endl;
        std::cout << "Total conflicts: " << currentConflicts_.size() << std::endl;
        std::cout << "Iterations: " << iterationCount_ << std::endl;
        std::cout << "Nodes explored: " << totalNodesExplored_ << std::endl;
        std::cout << "Planning time: " << totalPlanningTime_ << " seconds" << std::endl;
        
        if (!currentConflicts_.empty()) {
            std::cout << "\nConflicts detected:" << std::endl;
            for (size_t i = 0; i < std::min(currentConflicts_.size(), size_t(5)); i++) {
                const auto& conflict = currentConflicts_[i];
                std::cout << "  Conflict " << i+1 << ": Drone " << conflict.drone1Id 
                         << " vs Drone " << conflict.drone2Id 
                         << " at time " << conflict.timeStep << std::endl;
            }
            if (currentConflicts_.size() > 5) {
                std::cout << "  ... and " << (currentConflicts_.size() - 5) << " more conflicts" << std::endl;
            }
        }
    }

    bool MultiDronePlanner::planWithPriorityBasedApproach() {
        currentPaths_.clear();
        currentPaths_.resize(drones_.size());
        
        for (size_t i = 0; i < drones_.size(); i++) {
            iterationCount_++;
            
            // 为当前无人机规划路径，考虑之前规划的路径作为预约
            updateReservations(currentPaths_, static_cast<int>(i));
            
            Path path = singlePlanner_->planPath(drones_[i]);
            totalNodesExplored_ += singlePlanner_->getNodesExplored();
            
            if (path.empty()) {
                lastErrorMessage_ = "Failed to find path for drone " + std::to_string(drones_[i].id);
                return false;
            }
            
            currentPaths_[i] = path;
            
            if (g_config.enableDebugOutput) {
                std::cout << "Planned path for drone " << drones_[i].id 
                         << " with " << path.size() << " waypoints" << std::endl;
            }
        }
        
        return true;
    }

    bool MultiDronePlanner::planWithIterativeApproach() {
        const int maxIterations = 10;
        
        for (int iter = 0; iter < maxIterations; iter++) {
            iterationCount_++;
            
            // 检测当前冲突
            auto conflicts = ConflictDetector::detectConflicts(currentPaths_);
            
            if (conflicts.empty()) {
                return true; // 没有冲突，成功
            }
            
            // 选择第一个冲突进行解决
            const auto& conflict = conflicts[0];
            
            // 重新规划涉及冲突的无人机路径
            std::vector<int> dronesInConflict = {conflict.drone1Id, conflict.drone2Id};
            
            for (int droneId : dronesInConflict) {
                // 找到无人机索引
                auto it = std::find_if(drones_.begin(), drones_.end(),
                    [droneId](const DroneInfo& drone) { return drone.id == droneId; });
                
                if (it != drones_.end()) {
                    size_t droneIndex = std::distance(drones_.begin(), it);
                    
                    // 更新预约（排除当前无人机）
                    updateReservations(currentPaths_, droneId);
                    
                    // 重新规划路径
                    Path newPath = singlePlanner_->planPath(*it);
                    totalNodesExplored_ += singlePlanner_->getNodesExplored();
                    
                    if (!newPath.empty()) {
                        currentPaths_[droneIndex] = newPath;
                    }
                }
            }
        }
        
        lastErrorMessage_ = "Failed to resolve all conflicts within iteration limit";
        return false;
    }

    void MultiDronePlanner::updateReservations(const Paths& paths, int excludeDroneId) {
        auto reservations = pathsToSpaceTimeReservations(paths, excludeDroneId);
        singlePlanner_->setReservations(reservations);
    }

    void MultiDronePlanner::sortDronesByPriority() {
        std::sort(drones_.begin(), drones_.end(),
            [](const DroneInfo& a, const DroneInfo& b) {
                return a.priority < b.priority; // 较小的数字表示较高的优先级
            });
    }

    std::unordered_set<SpaceTimePoint, SpaceTimePointHash>
    MultiDronePlanner::pathsToSpaceTimeReservations(const Paths& paths, int excludeDroneId) const {
        std::unordered_set<SpaceTimePoint, SpaceTimePointHash> reservations;
        
        for (size_t i = 0; i < paths.size(); i++) {
            if (i < drones_.size() && drones_[i].id == excludeDroneId) {
                continue; // 跳过被排除的无人机
            }
            
            const auto& path = paths[i];
            for (const auto& node : path) {
                if (node) {
                    reservations.emplace(node->point, node->timeStep);
                }
            }
        }
        
        return reservations;
    }

    bool MultiDronePlanner::validatePaths(const Paths& paths) const {
        // 检查每条路径是否有效
        for (size_t i = 0; i < paths.size(); i++) {
            if (paths[i].empty()) {
                return false;
            }
            
            // 检查起点和终点
            if (i < drones_.size()) {
                const auto& drone = drones_[i];
                if (paths[i].front()->point != drone.start || 
                    paths[i].back()->point != drone.goal) {
                    return false;
                }
            }
        }
        
        return true;
    }

    void MultiDronePlanner::reset() {
        currentPaths_.clear();
        currentConflicts_.clear();
        iterationCount_ = 0;
        totalNodesExplored_ = 0;
        totalPlanningTime_ = 0.0;
        lastErrorMessage_.clear();
        
        if (singlePlanner_) {
            singlePlanner_->clearReservations();
        }
    }

} // namespace DronePathfinding