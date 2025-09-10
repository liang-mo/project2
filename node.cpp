#include "node.h"
#include <cmath>
#include <algorithm>


namespace DronePathfinding {

	Node::Node(const Point3D& p, const Point3D& target, int droneId, int timeStep,
		double g, Direction dir, double safety)
		: point(p), parent(nullptr), gCost(g), timeStep(timeStep),
		direction(dir), droneId(droneId), safetyCost(safety) {
		hCost = calculateHeuristic(p, target) * g_config.weightH;
	}

	double Node::getFCost() const {
		return gCost + hCost + safetyCost;
	}

	bool Node::operator>(const Node& other) const {
		return getFCost() > other.getFCost();
	}

	bool Node::operator<(const Node& other) const {
		return getFCost() < other.getFCost();
	}

	double Node::calculateHeuristic(const Point3D& from, const Point3D& to) {
		return std::abs(to.x - from.x) + std::abs(to.y - from.y) + std::abs(to.z - from.z);
	}

	std::vector<Point3D> Node::getPath() const {
		std::vector<Point3D> path;
		const Node* current = this;

		while (current != nullptr) {
			path.push_back(current->point);
			current = current->parent.get();
		}

		std::reverse(path.begin(), path.end());
		return path;
	}

} // namespace DronePathfinding