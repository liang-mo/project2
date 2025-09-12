#include "config.h"
#include <nlohmann/json.hpp>

namespace DronePathfinding {

    // 全局配置实例
    GlobalConfig g_config;

    void initializeConfig() {
        // 使用默认值初始化
        g_config = GlobalConfig();
    }

    void setConfigFromJson(const std::string& jsonStr) {
        try {
            nlohmann::json j = nlohmann::json::parse(jsonStr);
            
            if (j.contains("weightH")) g_config.weightH = j["weightH"];
            if (j.contains("weightG")) g_config.weightG = j["weightG"];
            if (j.contains("weightSafety")) g_config.weightSafety = j["weightSafety"];
            if (j.contains("maxIterations")) g_config.maxIterations = j["maxIterations"];
            if (j.contains("maxNodesExplored")) g_config.maxNodesExplored = j["maxNodesExplored"];
            if (j.contains("timeLimit")) g_config.timeLimit = j["timeLimit"];
            if (j.contains("safetyDistance")) g_config.safetyDistance = j["safetyDistance"];
            if (j.contains("followingDistance")) g_config.followingDistance = j["followingDistance"];
            if (j.contains("enablePathSmoothing")) g_config.enablePathSmoothing = j["enablePathSmoothing"];
            if (j.contains("smoothingIterations")) g_config.smoothingIterations = j["smoothingIterations"];
            if (j.contains("enableDebugOutput")) g_config.enableDebugOutput = j["enableDebugOutput"];
            if (j.contains("saveIntermediateResults")) g_config.saveIntermediateResults = j["saveIntermediateResults"];
        }
        catch (const std::exception& e) {
            // 解析失败时使用默认配置
            initializeConfig();
        }
    }

    std::string getConfigAsJson() {
        nlohmann::json j;
        j["weightH"] = g_config.weightH;
        j["weightG"] = g_config.weightG;
        j["weightSafety"] = g_config.weightSafety;
        j["maxIterations"] = g_config.maxIterations;
        j["maxNodesExplored"] = g_config.maxNodesExplored;
        j["timeLimit"] = g_config.timeLimit;
        j["safetyDistance"] = g_config.safetyDistance;
        j["followingDistance"] = g_config.followingDistance;
        j["enablePathSmoothing"] = g_config.enablePathSmoothing;
        j["smoothingIterations"] = g_config.smoothingIterations;
        j["enableDebugOutput"] = g_config.enableDebugOutput;
        j["saveIntermediateResults"] = g_config.saveIntermediateResults;
        
        return j.dump(4);
    }

} // namespace DronePathfinding