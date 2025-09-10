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

		// �����ܴ���
		double getFCost() const;

		// �Ƚϲ�����
		bool operator>(const Node& other) const;
		bool operator<(const Node& other) const;

		// ��������ʽ����
		static double calculateHeuristic(const Point3D& from, const Point3D& to);

		// ��ȡ·��
		std::vector<Point3D> getPath() const;
	};

	// �ڵ�Ƚ������������ȶ��У�
	struct NodeComparator {
		bool operator()(const NodePtr& a, const NodePtr& b) const {
			return a->getFCost() > b->getFCost();
		}
	};

} // namespace DronePathfinding