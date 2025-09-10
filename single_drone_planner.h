#pragma once

#include "types.h"
#include "map3d.h"
#include "node.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>


namespace DronePathfinding {

	class SingleDronePlanner {
	private:
		const Map3D& map_;
		std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> openList_;
		std::vector<NodePtr> closedList_;
		std::unordered_map<Point3D, NodePtr, Point3DHash> visitedNodes_;

		// ʱ��Լ�����������˻���ռ����Ϣ��
		std::unordered_set<SpaceTimePoint, SpaceTimePointHash> reservedSpaceTime_;

	public:
		explicit SingleDronePlanner(const Map3D& map);

		// ����ʱ��Լ��
		void setReservedSpaceTime(const std::unordered_set<SpaceTimePoint, SpaceTimePointHash>& reserved);
		void addReservedSpaceTime(const Point3D& point, int timeStep);
		void clearReservedSpaceTime();
		// ·���滮
		Path planPath(const DroneInfo& drone);

		// ��ȫ���
		bool isPassable(const Point3D& point) const;
		bool isSpaceTimeFree(const Point3D& point, int timeStep) const;
		int calculateSafetyLevel(const Point3D& point) const;

	private:
		// �������
		void searchNeighbors(NodePtr currentNode, const Point3D& target, int droneId);
		void addNeighbor(NodePtr parent, const Point3D& point, const Point3D& target,
			int timeStep, Direction neighborDirection, int droneId);

		// ���߷���
		bool checkSafetyDistance(const Point3D& point, int safetyDistance) const;
		double calculateStepWeight(Direction prevDir, Direction currDir) const;
		std::vector<std::pair<Point3D, Direction>> getNeighborDirections(const Point3D& point, Direction currentDir) const;

		// ������
		void reset();
	};

} // namespace DronePathfinding