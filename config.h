#pragma once

#include "types.h"

namespace DronePathfinding {

    // 全局配置结构
    struct GlobalConfig {
        // A*算法参数
        double weightH = 1.0;           // 启发式权重
        double weightG = 1.0;           // 实际代价权重
        double weightSafety = 0.5;      // 安全代价权重
        
        // 搜索参数
        int maxIterations = 10000;      // 最大迭代次数
        int maxNodesExplored = 50000;   // 最大探索节点数
        double timeLimit = 30.0;        // 时间限制(秒)
        
        // 冲突检测参数
        double safetyDistance = 2.0;    // 安全距离
        double followingDistance = 3.0; // 跟随距离
        
        // 路径平滑参数
        bool enablePathSmoothing = true;
        int smoothingIterations = 3;
        
        // 调试参数
        bool enableDebugOutput = false;
        bool saveIntermediateResults = false;
    };

    // 全局配置实例
    extern GlobalConfig g_config;

    // 配置管理函数
    void initializeConfig();
    void setConfigFromJson(const std::string& jsonStr);
    std::string getConfigAsJson();

} // namespace DronePathfinding