#pragma once

#include "types.h"
#include "config.h"
#include <memory>


namespace DronePathfinding {

	class Node {
	public:
		Point3D point;
		std::shared_ptr<Node> parent;
		double gCost;
		double hCost;
		double safetyCost;
		int timeStep;
		Direction direction;
		int droneId;

		Node(const Point3D& p, const Point3D& target, int droneId, int timeStep = 0,
			double g = 0.0, Direction dir = Direction::ORIGIN, double safety = 0.0);

		// 计算总代价
		double getFCost() const;

		// 比较操作符
		bool operator>(const Node& other) const;
		bool operator<(const Node& other) const;

		// 计算启发式距离
		static double calculateHeuristic(const Point3D& from, const Point3D& to);

		// 获取路径
		std::vector<Point3D> getPath() const;
	};

	// 节点比较器（用于优先队列）
	struct NodeComparator {
		bool operator()(const NodePtr& a, const NodePtr& b) const {
			return a->getFCost() > b->getFCost();
		}
	};

} // namespace DronePathfinding