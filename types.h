#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <tuple>
#include <functional>


namespace DronePathfinding {

	// 前向声明
	class Node;

	// 基础类型定义
	using NodePtr = std::shared_ptr<Node>;
	using Path = std::vector<NodePtr>;
	using Paths = std::vector<Path>;

	// 三维点结构
	struct Point3D {
		int x, y, z;

		Point3D() : x(0), y(0), z(0) {}
		Point3D(int x, int y, int z) : x(x), y(y), z(z) {}

		bool operator==(const Point3D& other) const {
			return x == other.x && y == other.y && z == other.z;
		}

		bool operator!=(const Point3D& other) const {
			return !(*this == other);
		}

		Point3D operator+(const Point3D& other) const {
			return Point3D(x + other.x, y + other.y, z + other.z);
		}

		Point3D operator-(const Point3D& other) const {
			return Point3D(x - other.x, y - other.y, z - other.z);
		}
	};

	// 无人机信息结构
	struct DroneInfo {
		int id;
		Point3D startPoint;
		Point3D endPoint;
		double priority;        // 优先级，数值越高优先级越高
		double maxSpeed;        // 最大速度
		double safetyRadius;    // 安全半径

		DroneInfo(int id, const Point3D& start, const Point3D& end,
			double priority = 1.0, double maxSpeed = 1.0, double safetyRadius = 1.0)
			: id(id), startPoint(start), endPoint(end),
			priority(priority), maxSpeed(maxSpeed), safetyRadius(safetyRadius) {
		}
	};

	// 方向枚举
	enum class Direction {
		N = 0, EN = 1, E = 2, ES = 3, S = 4, WS = 5, W = 6, WN = 7,
		UN = 8, UEN = 9, UE = 10, UES = 11, US = 12, UWS = 13, UW = 14, UWN = 15,
		DN = 16, DEN = 17, DE = 18, DES = 19, DS = 20, DWS = 21, DW = 22, DWN = 23,
		ORIGIN = -1
	};

	// 冲突类型
	enum class ConflictType {
		NONE,
		VERTEX,     // 顶点冲突（同一时间在同一位置）
		EDGE,       // 边冲突（交换位置）
		FOLLOWING   // 跟随冲突（距离过近）
	};

	// 冲突信息
	struct Conflict {
		ConflictType type;
		int droneId1, droneId2;
		int timeStep;
		Point3D location1, location2;

		Conflict(ConflictType t, int id1, int id2, int time,
			const Point3D& loc1, const Point3D& loc2 = Point3D())
			: type(t), droneId1(id1), droneId2(id2), timeStep(time),
			location1(loc1), location2(loc2) {
		}
	};

	// 哈希函数
	struct Point3DHash {
		std::size_t operator()(const Point3D& p) const {
			return std::hash<int>()(p.x) ^
				(std::hash<int>()(p.y) << 1) ^
				(std::hash<int>()(p.z) << 2);
		}
	};

	struct TupleHash {
		std::size_t operator()(const std::tuple<int, int, int>& t) const {
			auto h1 = std::hash<int>{}(std::get<0>(t));
			auto h2 = std::hash<int>{}(std::get<1>(t));
			auto h3 = std::hash<int>{}(std::get<2>(t));
			return h1 ^ (h2 << 1) ^ (h3 << 2);
		}
	};

	// 时空点（包含时间维度的点）
	struct SpaceTimePoint {
		Point3D point;
		int timeStep;

		SpaceTimePoint(const Point3D& p, int t) : point(p), timeStep(t) {}

		bool operator==(const SpaceTimePoint& other) const {
			return point == other.point && timeStep == other.timeStep;
		}
	};

	struct SpaceTimePointHash {
		std::size_t operator()(const SpaceTimePoint& stp) const {
			return Point3DHash()(stp.point) ^ (std::hash<int>()(stp.timeStep) << 3);
		}
	};

} // namespace DronePathfinding

