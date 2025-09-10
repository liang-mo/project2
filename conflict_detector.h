#pragma once

#include "types.h"
#include <vector>


namespace DronePathfinding {

	class ConflictDetector {
	public:
		// �������·��֮��ĳ�ͻ
		static std::vector<Conflict> detectConflicts(const Paths& paths);

		// �������·��֮��ĳ�ͻ
		static std::vector<Conflict> detectConflictsBetweenPaths(const Path& path1, const Path& path2);

		// ����ض����͵ĳ�ͻ
		static std::vector<Conflict> detectVertexConflicts(const Paths& paths);
		static std::vector<Conflict> detectEdgeConflicts(const Paths& paths);
		static std::vector<Conflict> detectFollowingConflicts(const Paths& paths);

	private:
		// ��������
		static bool isVertexConflict(const NodePtr& node1, const NodePtr& node2);
		static bool isEdgeConflict(const NodePtr& node1a, const NodePtr& node1b,
			const NodePtr& node2a, const NodePtr& node2b);
		static bool isFollowingConflict(const NodePtr& node1, const NodePtr& node2, double minDistance);
		static double calculateDistance(const Point3D& p1, const Point3D& p2);
	};

} // namespace DronePathfinding