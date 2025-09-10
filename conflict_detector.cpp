#include "conflict_detector.h"
#include "node.h"
#include <cmath>


namespace DronePathfinding {

	std::vector<Conflict> ConflictDetector::detectConflicts(const Paths& paths) {
		std::vector<Conflict> conflicts;

		auto vertexConflicts = detectVertexConflicts(paths);
		auto edgeConflicts = detectEdgeConflicts(paths);
		auto followingConflicts = detectFollowingConflicts(paths);

		conflicts.insert(conflicts.end(), vertexConflicts.begin(), vertexConflicts.end());
		conflicts.insert(conflicts.end(), edgeConflicts.begin(), edgeConflicts.end());
		conflicts.insert(conflicts.end(), followingConflicts.begin(), followingConflicts.end());

		return conflicts;
	}

	std::vector<Conflict> ConflictDetector::detectConflictsBetweenPaths(const Path& path1, const Path& path2) {
		std::vector<Conflict> conflicts;

		if (path1.empty() || path2.empty()) {
			return conflicts;
		}

		int maxTime = std::max(static_cast<int>(path1.size()), static_cast<int>(path2.size()));

		for (int timeStep = 0; timeStep < maxTime - 1; timeStep++) {
			// Get current time step nodes
			NodePtr node1_current = (timeStep < static_cast<int>(path1.size())) ? path1[timeStep] : path1.back();
			NodePtr node2_current = (timeStep < static_cast<int>(path2.size())) ? path2[timeStep] : path2.back();

			// Check vertex conflicts
			if (isVertexConflict(node1_current, node2_current)) {
				conflicts.emplace_back(ConflictType::VERTEX, node1_current->droneId, node2_current->droneId,
					timeStep, node1_current->point, node2_current->point);
			}

			// Check edge conflicts
			if (timeStep < maxTime - 1) {
				NodePtr node1_next = (timeStep + 1 < static_cast<int>(path1.size())) ? path1[timeStep + 1] : path1.back();
				NodePtr node2_next = (timeStep + 1 < static_cast<int>(path2.size())) ? path2[timeStep + 1] : path2.back();

				if (isEdgeConflict(node1_current, node1_next, node2_current, node2_next)) {
					conflicts.emplace_back(ConflictType::EDGE, node1_current->droneId, node2_current->droneId,
						timeStep, node1_current->point, node2_current->point);
				}
			}

			// Check following conflicts
			if (isFollowingConflict(node1_current, node2_current, 2.0)) {
				conflicts.emplace_back(ConflictType::FOLLOWING, node1_current->droneId, node2_current->droneId,
					timeStep, node1_current->point, node2_current->point);
			}
		}

		return conflicts;
	}

	std::vector<Conflict> ConflictDetector::detectVertexConflicts(const Paths& paths) {
		std::vector<Conflict> conflicts;

		for (size_t i = 0; i < paths.size(); i++) {
			for (size_t j = i + 1; j < paths.size(); j++) {
				auto pathConflicts = detectConflictsBetweenPaths(paths[i], paths[j]);
				for (const auto& conflict : pathConflicts) {
					if (conflict.type == ConflictType::VERTEX) {
						conflicts.push_back(conflict);
					}
				}
			}
		}

		return conflicts;
	}

	std::vector<Conflict> ConflictDetector::detectEdgeConflicts(const Paths& paths) {
		std::vector<Conflict> conflicts;

		for (size_t i = 0; i < paths.size(); i++) {
			for (size_t j = i + 1; j < paths.size(); j++) {
				auto pathConflicts = detectConflictsBetweenPaths(paths[i], paths[j]);
				for (const auto& conflict : pathConflicts) {
					if (conflict.type == ConflictType::EDGE) {
						conflicts.push_back(conflict);
					}
				}
			}
		}

		return conflicts;
	}

	std::vector<Conflict> ConflictDetector::detectFollowingConflicts(const Paths& paths) {
		std::vector<Conflict> conflicts;

		for (size_t i = 0; i < paths.size(); i++) {
			for (size_t j = i + 1; j < paths.size(); j++) {
				auto pathConflicts = detectConflictsBetweenPaths(paths[i], paths[j]);
				for (const auto& conflict : pathConflicts) {
					if (conflict.type == ConflictType::FOLLOWING) {
						conflicts.push_back(conflict);
					}
				}
			}
		}

		return conflicts;
	}

	bool ConflictDetector::isVertexConflict(const NodePtr& node1, const NodePtr& node2) {
		return node1->point == node2->point && node1->timeStep == node2->timeStep;
	}

	bool ConflictDetector::isEdgeConflict(const NodePtr& node1a, const NodePtr& node1b,
		const NodePtr& node2a, const NodePtr& node2b) {
		return (node1a->point == node2b->point && node1b->point == node2a->point);
	}

	bool ConflictDetector::isFollowingConflict(const NodePtr& node1, const NodePtr& node2, double minDistance) {
		double distance = calculateDistance(node1->point, node2->point);
		return distance < minDistance && node1->timeStep == node2->timeStep;
	}

	double ConflictDetector::calculateDistance(const Point3D& p1, const Point3D& p2) {
		double dx = static_cast<double>(p1.x - p2.x);
		double dy = static_cast<double>(p1.y - p2.y);
		double dz = static_cast<double>(p1.z - p2.z);
		return std::sqrt(dx * dx + dy * dy + dz * dz);
	}

} // namespace DronePathfinding