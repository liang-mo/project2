#include <iostream>
#include <string>
#include <vector>
#include <libpq-fe.h>
#include "VoxelMap3D.h"
#include "map3d.h"
#include "multi_drone_planner.h"
#include "config.h"

using namespace DronePathfinding;

int main() {
    std::cout << "=== 多无人机路径规划测试 ===" << std::endl;
    
    // 1. 初始化配置
    initializeConfig();
    g_config.enableDebugOutput = true;
    g_config.maxIterations = 5000;
    g_config.safetyDistance = 2.0;
    
    // 2. 数据库连接
    const char* conninfo = "host=10.12.201.6 port=5432 dbname=postgres user=postgres password=postgres";
    PGconn* conn = PQconnectdb(conninfo);

    if (PQstatus(conn) != CONNECTION_OK) {
        std::cerr << "❌ 数据库连接失败: " << PQerrorMessage(conn) << std::endl;
        if (conn) PQfinish(conn);
        return -1;
    }
    std::cout << "✅ 数据库连接成功" << std::endl;

    // 3. 加载地图数据
    std::string sql = 
        "SELECT id, states, code3d "
        "FROM public.shaoxin_l7a6_fly_grid "
        "ORDER BY id ASC LIMIT 1000";  // 限制数据量以便测试

    Map3D map;
    if (!map.loadFromDatabase(conn, sql)) {
        std::cerr << "❌ 地图加载失败" << std::endl;
        PQfinish(conn);
        return -1;
    }
    
    std::cout << "✅ 地图加载成功" << std::endl;
    map.printStatistics();

    // 4. 创建多无人机规划器
    MultiDronePlanner planner(map);

    // 5. 添加测试无人机
    std::vector<DroneInfo> testDrones = {
        DroneInfo(1, Point3D{10, 10, 5}, Point3D{50, 50, 5}, 1.0, 2.0, 1),
        DroneInfo(2, Point3D{15, 10, 5}, Point3D{45, 50, 5}, 1.0, 2.0, 2),
        DroneInfo(3, Point3D{10, 15, 5}, Point3D{50, 45, 5}, 1.0, 2.0, 3),
        DroneInfo(4, Point3D{20, 20, 5}, Point3D{40, 40, 5}, 1.0, 2.0, 4)
    };

    std::cout << "\n=== 添加测试无人机 ===" << std::endl;
    for (const auto& drone : testDrones) {
        planner.addDrone(drone);
        std::cout << "添加无人机 " << drone.id 
                  << ": 起点(" << drone.start.x << "," << drone.start.y << "," << drone.start.z << ")"
                  << " -> 终点(" << drone.goal.x << "," << drone.goal.y << "," << drone.goal.z << ")"
                  << " 优先级:" << drone.priority << std::endl;
    }

    // 6. 执行路径规划
    std::cout << "\n=== 开始路径规划 ===" << std::endl;
    auto startTime = std::chrono::high_resolution_clock::now();
    
    bool success = planner.planPaths();
    
    auto endTime = std::chrono::high_resolution_clock::now();
    double totalTime = std::chrono::duration<double>(endTime - startTime).count();

    // 7. 输出结果
    std::cout << "\n=== 规划结果 ===" << std::endl;
    if (success) {
        std::cout << "✅ 路径规划成功！" << std::endl;
        
        const auto& paths = planner.getPaths();
        std::cout << "成功规划了 " << paths.size() << " 条路径" << std::endl;
        
        // 显示每条路径的基本信息
        for (size_t i = 0; i < paths.size(); i++) {
            const auto& path = paths[i];
            if (!path.empty()) {
                std::cout << "无人机 " << (i+1) << " 路径长度: " << path.size() 
                         << " 步，总时间: " << (path.back()->timeStep) << " 时间步" << std::endl;
                
                // 显示前几个路径点
                std::cout << "  路径点: ";
                for (size_t j = 0; j < std::min(path.size(), size_t(5)); j++) {
                    const auto& point = path[j]->point;
                    std::cout << "(" << point.x << "," << point.y << "," << point.z << ")";
                    if (j < std::min(path.size(), size_t(5)) - 1) std::cout << " -> ";
                }
                if (path.size() > 5) {
                    std::cout << " -> ... -> (" << path.back()->point.x 
                             << "," << path.back()->point.y 
                             << "," << path.back()->point.z << ")";
                }
                std::cout << std::endl;
            }
        }
        
        // 检查冲突
        const auto& conflicts = planner.getConflicts();
        if (conflicts.empty()) {
            std::cout << "✅ 无冲突检测到" << std::endl;
        } else {
            std::cout << "⚠️  检测到 " << conflicts.size() << " 个冲突" << std::endl;
            for (size_t i = 0; i < std::min(conflicts.size(), size_t(3)); i++) {
                const auto& conflict = conflicts[i];
                std::cout << "  冲突 " << (i+1) << ": 无人机 " << conflict.drone1Id 
                         << " 与无人机 " << conflict.drone2Id 
                         << " 在时间步 " << conflict.timeStep << std::endl;
            }
        }
        
    } else {
        std::cout << "❌ 路径规划失败: " << planner.getLastError() << std::endl;
    }

    // 8. 输出统计信息
    planner.printStatistics();
    std::cout << "总耗时: " << totalTime << " 秒" << std::endl;

    // 9. 保存结果到JSON文件
    if (success) {
        try {
            planner.savePathsToJson("multi_drone_paths.json");
            std::cout << "✅ 路径结果已保存到 multi_drone_paths.json" << std::endl;
        } catch (const std::exception& e) {
            std::cout << "⚠️  保存JSON文件失败: " << e.what() << std::endl;
        }
    }

    // 10. 测试不同场景
    std::cout << "\n=== 测试简化场景 ===" << std::endl;
    
    // 创建一个简化的测试场景（不依赖数据库）
    MultiDronePlanner simplePlanner(map);
    
    // 添加两个简单的无人机
    DroneInfo drone1(1, Point3D{0, 0, 0}, Point3D{5, 5, 0}, 1.0, 1.0, 1);
    DroneInfo drone2(2, Point3D{5, 0, 0}, Point3D{0, 5, 0}, 1.0, 1.0, 2);
    
    simplePlanner.addDrone(drone1);
    simplePlanner.addDrone(drone2);
    
    std::cout << "简化场景规划中..." << std::endl;
    bool simpleSuccess = simplePlanner.planPaths();
    
    if (simpleSuccess) {
        std::cout << "✅ 简化场景规划成功" << std::endl;
        simplePlanner.printStatistics();
    } else {
        std::cout << "❌ 简化场景规划失败: " << simplePlanner.getLastError() << std::endl;
    }

    // 11. 清理资源
    PQfinish(conn);
    std::cout << "\n=== 测试完成 ===" << std::endl;
    
    return success ? 0 : 1;
}