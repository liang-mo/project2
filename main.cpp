//#include <iostream>
//#include <libpq-fe.h>
//#include <regex>
//#include <string>
//#include <cmath>
//#include "map3d.h"
//#include "cbs_planner.h"
//#include "multi_drone_planner.h"
//#include "Map3DManager.h"
//#include "config.h"
//#include <vector>
//#include <chrono>
//#include <thread>
//#include <mutex>
//#include <fstream>
//#include <iomanip> 
//
//using namespace DronePathfinding;
//using namespace std;
//#include "crow_all.h"
//int crow::detail::dumb_timer_queue::tick = 0;
//
//namespace crow {
//	std::unordered_map<std::string, std::string> mime_types = {
//		{"html", "text/html"},
//		{"htm", "text/html"},
//		{"css", "text/css"},
//		{"js", "application/javascript"},
//		{"png", "image/png"},
//		{"jpg", "image/jpeg"},
//		{"jpeg", "image/jpeg"},
//		{"gif", "image/gif"},
//		{"svg", "image/svg+xml"},
//		{"ico", "image/x-icon"},
//		{"json", "application/json"},
//		{"txt", "text/plain"},
//		{"pdf", "application/pdf"},
//		{"zip", "application/zip"},
//		{"gz", "application/gzip"},
//		{"mp3", "audio/mpeg"},
//		{"mp4", "video/mp4"},
//		{"xml", "application/xml"}
//
//	};
//}
//
//
//
//// 使用 SingleDronePlanner::isPassable 来找到最近的可通行点
//Point3D findNearestPassablePoint(const Point3D& start, SingleDronePlanner& planner) {
//    if (planner.isPassable(start)) {
//        return start; // 如果本身可通行，直接返回
//    }
//
//    
//    const std::vector<Point3D> directions = {
//        {1,0,0}, {-1,0,0},
//        {0,1,0}, {0,-1,0},
//        {0,0,1}, {0,0,-1}
//    };
//
//    std::queue<Point3D> q;
//    std::unordered_set<Point3D, Point3DHash> visited;
//    q.push(start);
//    visited.insert(start);
//
//    while (!q.empty()) {
//        Point3D cur = q.front();
//        q.pop();
//
//        for (const auto& d : directions) {
//            Point3D next(cur.x + d.x, cur.y + d.y, cur.z + d.z);
//            if (visited.count(next)) continue;
//            visited.insert(next);
//
//            if (planner.isPassable(next)) {
//                return next; // 找到可通行点
//            }
//            q.push(next);
//        }
//    }
//
//    throw std::runtime_error("找不到附近可通行点");
//}
//
//
//
//
//
//
//using namespace std;
//
//
//double roundToPrecision(double value, int precision) {
//    double scale = std::pow(10, precision);
//    return std::round(value * scale) / scale;
//}
//int main() {
//    crow::SimpleApp app;
//
//    // --- Step 1: 初始化地图数据 ---
//    CROW_ROUTE(app, "/init").methods("GET"_method)
//        ([](const crow::request& req) {
//        const char* conninfo = "host=10.12.201.6 port=5432 dbname=postgres user=postgres password=postgres";
//        PGconn* conn = PQconnectdb(conninfo);
//
//        if (PQstatus(conn) != CONNECTION_OK) {
//            std::cerr << "数据库连接失败: " << PQerrorMessage(conn) << std::endl;
//            PQfinish(conn);
//            return crow::response(500, "数据库连接失败");
//        }
//        cout << "数据库连接成功" << endl;
//
//        // 固定 SQL
//        std::ostringstream sql;
//        sql << std::fixed << std::setprecision(9);
//        sql << "SELECT pointt,code3d,state "
//            << "FROM public.shao_xin_fly_grid "
//            << "WHERE ST_3DIntersects("
//            << "geom3d, "
//            << "ST_3DMakeBox("
//            << "ST_POINTZ(120.540243056, 29.991770833, 21.670386077, 4326), "
//            << "ST_POINTZ(120.560312500, 30.006840278, 105.972605184, 4326)"
//            << "));";
//
//        try {
//            // 加载到 Map3DManager 中
//            Map3DManager::getInstance().loadFromDatabase(conn, sql.str());
//            std::cout << "地图加载成功！" << std::endl;
//        }
//        catch (const std::exception& ex) {
//            PQfinish(conn);
//            return crow::response(500, std::string("地图加载失败: ") + ex.what());
//        }
//
//        PQfinish(conn);
//        return crow::response{ R"({"status": "地图加载成功"})" };
//            });
//
//    // --- Step 2: 使用已有地图进行路径规划 ---
//    CROW_ROUTE(app, "/plan").methods("POST"_method)
//        ([](const crow::request& req) {
//        std::cout << "=== 收到 /plan 请求 ===\n";
//        std::cout << "Body:\n" << req.body << "\n";
//
//        nlohmann::json body;
//        try {
//            body = nlohmann::json::parse(req.body);
//        }
//        catch (const std::exception& e) {
//            return crow::response(400, std::string("Invalid JSON: ") + e.what());
//        }
//
//        if (!body.contains("odPairs") || !body["odPairs"].is_array() || body["odPairs"].empty()) {
//            return crow::response(400, "Missing odPairs array");
//        }
//
//        auto map = Map3DManager::getInstance().getMap();
//        if (!map) {
//            return crow::response(400, "Map not initialized - call /init first");
//        }
//
//        int precision = body.value("precision", 9);
//
//        
//        // bufferRadius -> preferredSafetyDistance
//        if (body.contains("bufferRadius")) {
//            
//            DronePathfinding::g_config.preferredSafetyDistance = 1;
//                
//            std::cout << "[Config] preferredSafetyDistance = "
//                << DronePathfinding::g_config.preferredSafetyDistance << "\n";
//        }
//
//        // uavType 影响四个方向权重
//        if (body.contains("uavType")) {
//            int uavType = body.value("uavType", 2);
//            if (uavType == 1) {
//                DronePathfinding::g_config.weightStraight = 0.9;
//                DronePathfinding::g_config.weightVertical = 3.0;
//                DronePathfinding::g_config.weightHorizontal = 2.5;
//                DronePathfinding::g_config.weightDiagonal = 1.8;
//            }
//            else if (uavType == 2) {
//                DronePathfinding::g_config.weightStraight = 1;
//                DronePathfinding::g_config.weightVertical = 1.0;
//                DronePathfinding::g_config.weightHorizontal = 1.0;
//                DronePathfinding::g_config.weightDiagonal = 1.2;
//            }
//            else if (uavType == 3) {
//                DronePathfinding::g_config.weightStraight = 1;
//                DronePathfinding::g_config.weightVertical = 1.2;
//                DronePathfinding::g_config.weightHorizontal = 1.1;
//                DronePathfinding::g_config.weightDiagonal = 1.3;
//            }
//            else if (uavType == 4) {
//                DronePathfinding::g_config.weightStraight = 1.1;
//                DronePathfinding::g_config.weightVertical = 1.3;
//                DronePathfinding::g_config.weightHorizontal = 1.1;
//                DronePathfinding::g_config.weightDiagonal = 1.4;
//            }
//            
//        }
//        
//
//        // 创建多无人机规划器
//        MultiDronePlanner planner(*map);
//
//        // 读取每个无人机的起点和终点
//        int droneId = 1;
//        int weight = 10;
//        for (const auto& od : body["odPairs"]) {
//            if (!od.contains("origin") || !od.contains("destination")) {
//                return crow::response(400, "Each odPair must contain origin and destination");
//            }
//
//            auto origin = od["origin"];
//            auto destination = od["destination"];
//
//            double olon = roundToPrecision(origin.value("longitude", 0.0), 9);
//            double olat = roundToPrecision(origin.value("latitude", 0.0), 9);
//            double oalt = roundToPrecision(origin.value("altitude", 0.0), 9);
//
//            double dlon = roundToPrecision(destination.value("longitude", 0.0), 9);
//            double dlat = roundToPrecision(destination.value("latitude", 0.0), 9);
//            double dalt = roundToPrecision(destination.value("altitude", 0.0), 9);
//
//            try {
//                auto originIdx = map->getIndexFromCoordinate(olon, olat, oalt, precision);
//                auto destIdx = map->getIndexFromCoordinate(dlon, dlat, dalt, precision);
//                Point3D start = Point3D(std::get<0>(originIdx), std::get<1>(originIdx), std::get<2>(originIdx));
//                Point3D end = Point3D(std::get<0>(destIdx), std::get<1>(destIdx), std::get<2>(destIdx));
//				SingleDronePlanner singlePlanner(*map);
//				start = findNearestPassablePoint(start, singlePlanner);
//				end = findNearestPassablePoint(end, singlePlanner);
//
//                DroneInfo drone(
//                    droneId++,
//                    start,
//                    end,
//                    weight-- 
//                );
//                planner.addDrone(drone);
//            }
//            catch (const std::out_of_range& e) {
//                return crow::response(400, std::string("Coordinate not found in grid: ") + e.what());
//            }
//        }
//        try{
//            auto start = std::chrono::high_resolution_clock::now();
//            bool success = planner.planPaths();
//            auto end = std::chrono::high_resolution_clock::now();
//            double timeUsed = std::chrono::duration<double>(end - start).count();
//
//            nlohmann::json responseJson;
//            responseJson["success"] = success;
//            responseJson["time_used"] = timeUsed;
//            if (success) {
//                responseJson["result"] = planner.getPathsAsJson();
//                return crow::response(200, responseJson.dump(4));
//            }
//            else {
//                responseJson["error"] = planner.getLastError();
//                return crow::response(500, responseJson.dump(4));
//            }
//        }
//        catch (const std::exception& e) {
//            nlohmann::json err;
//            err["success"] = false;
//            err["error"] = std::string("后端异常: ") + e.what();
//            return crow::response(500, err.dump(4));
//        }
//        catch (...) {
//            nlohmann::json err;
//            err["success"] = false;
//            err["error"] = "未知异常";
//            return crow::response(500, err.dump(4));
//        }
//
//        
//
//        
//
//    
//            });
//
//
//    std::cout << "Crow HTTP 服务运行中：http://10.12.201.46:18080\n";
//    app.bindaddr("0.0.0.0").port(18080).multithreaded().run();
//
//    return 0;
//}
//
