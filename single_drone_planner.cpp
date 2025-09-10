#include "single_drone_planner.h"
#include "config.h"
#include <iostream>
#include <algorithm>


namespace DronePathfinding {

	SingleDronePlanner::SingleDronePlanner(const Map3D& map) : map_(map) {}

	void SingleDronePlanner::setReservedSpaceTime(const std::unordered_set<SpaceTimePoint, SpaceTimePointHash>& reserved) {
		reservedSpaceTime_ = reserved;
	}

	void SingleDronePlanner::addReservedSpaceTime(const Point3D& point, int timeStep) {
		reservedSpaceTime_.emplace(point, timeStep);
	}

	void SingleDronePlanner::clearReservedSpaceTime() {
		reservedSpaceTime_.clear();
	}



	Path SingleDronePlanner::planPath(const DroneInfo& drone) {
		reset();

		// 检查起点和终点的有效性
		if (!isPassable(drone.startPoint)) {
			std::cout << "警告：无人机 " << drone.id << " 起点不可通行" << std::endl;
		}

		if (!isPassable(drone.endPoint)) {
			std::cout << "警告：无人机 " << drone.id << " 终点不可通行" << std::endl;
		}

		// 创建起始节点
		int startSafetyLevel = calculateSafetyLevel(drone.startPoint);
		auto startNode = std::make_shared<Node>(drone.startPoint, drone.endPoint, drone.id, 0,
			0.0, Direction::ORIGIN, startSafetyLevel * g_config.weightSafety);

		openList_.push(startNode);
		visitedNodes_[drone.startPoint] = startNode;

		int nodesExplored = 0;

		while (!openList_.empty() && nodesExplored < g_config.maxNodesExplored) {
			auto currentNode = openList_.top();
			openList_.pop();

			nodesExplored++;
			closedList_.push_back(currentNode);

			// 检查是否到达目标
			if (currentNode->point == drone.endPoint) {
				std::cout << "无人机 " << drone.id << " 找到路径，探索节点数: " << nodesExplored << std::endl;

				// 构建路径
				Path path;
				auto node = currentNode;
				while (node != nullptr) {
					path.push_back(node);
					node = node->parent;
				}
				std::reverse(path.begin(), path.end());
				return path;
			}

			// 搜索邻居节点
			searchNeighbors(currentNode, drone.endPoint, drone.id);
		}

		std::cout << "无人机 " << drone.id << " 未找到路径，探索节点数: " << nodesExplored << std::endl;
		return {};
	}

	bool SingleDronePlanner::isPassable(const Point3D& point) const {
		if (!map_.isValidIndex(point.z, point.x, point.y)) {
			return false;
		}
		
		if (map_(point.z, point.x, point.y) != g_config.passableTag) {
			return false;
		}
		
		return checkSafetyDistance(point, g_config.minSafetyDistance);
	}

	bool SingleDronePlanner::isSpaceTimeFree(const Point3D& point, int timeStep) const {
		SpaceTimePoint stp(point, timeStep);
		return reservedSpaceTime_.find(stp) == reservedSpaceTime_.end();
	}

	

	int SingleDronePlanner::calculateSafetyLevel(const Point3D& point) const {
		int dist = g_config.preferredSafetyDistance + 1;

		// 找到当前点能保证的最大安全距离
		for (int d = g_config.preferredSafetyDistance; d >= g_config.minSafetyDistance; d--) {
			if (checkSafetyDistance(point, d)) {
				dist = d;
				break;
			}
		}

		// 越靠近障碍物，惩罚越大
		int penalty = (g_config.preferredSafetyDistance - dist + 1);
		return penalty * penalty * penalty;
	}

	void SingleDronePlanner::searchNeighbors(NodePtr currentNode, const Point3D& target, int droneId) {
		auto neighbors = getNeighborDirections(currentNode->point, currentNode->direction);

		for (const auto& neighbor : neighbors) {
			Point3D neighborPoint = neighbor.first;
			Direction neighborDirection = neighbor.second;

			if (!map_.isValidIndex(neighborPoint.z, neighborPoint.x, neighborPoint.y)) {
				continue;
			}

			if (!isPassable(neighborPoint)) {
				continue;
			}

			// 检查时空冲突
			int nextTimeStep = currentNode->timeStep + 1;
			if (!isSpaceTimeFree(neighborPoint, nextTimeStep)) {
				continue;
			}

			// 检查是否在关闭列表中
			bool inClosedList = false;
			for (const auto& closedNode : closedList_) {
				if (closedNode->point == neighborPoint) {
					inClosedList = true;
					break;
				}
			}
			if (inClosedList) {
				continue;
			}

			addNeighbor(currentNode, neighborPoint, target, nextTimeStep, neighborDirection, droneId);
		}
	}

	void SingleDronePlanner::addNeighbor(NodePtr parent, const Point3D& point, const Point3D& target,
		int timeStep, Direction neighborDirection, int droneId) {
		int safetyLevel = calculateSafetyLevel(point);
		double stepWeight = calculateStepWeight(parent->direction, neighborDirection) * g_config.weightG;
		double newG = parent->gCost + stepWeight;
		double safetyCost = safetyLevel * g_config.weightSafety;

		auto it = visitedNodes_.find(point);
		if (it == visitedNodes_.end()) {
			// 创建新节点
			auto newNode = std::make_shared<Node>(point, target, droneId, timeStep, newG, neighborDirection, safetyCost);
			newNode->parent = parent;
			openList_.push(newNode);
			visitedNodes_[point] = newNode;
		}
		else {
			// 更新现有节点
			auto existingNode = it->second;
			auto tempNode = std::make_shared<Node>(point, target, droneId, timeStep, newG, neighborDirection, safetyCost);

			if (tempNode->getFCost() < existingNode->getFCost()) {
				existingNode->gCost = newG;
				existingNode->parent = parent;
				existingNode->safetyCost = safetyCost;
				existingNode->direction = neighborDirection;
				existingNode->timeStep = timeStep;
			}
		}
	}

	bool SingleDronePlanner::checkSafetyDistance(const Point3D& point, int safetyDistance) const {
		for (int dx = -safetyDistance; dx <= safetyDistance; dx++) {
			for (int dy = -safetyDistance; dy <= safetyDistance; dy++) {
				for (int dz = -safetyDistance; dz <= safetyDistance; dz++) {
					Point3D checkPoint(point.x + dx, point.y + dy, point.z + dz);

					if (map_.isValidIndex(checkPoint.z, checkPoint.x, checkPoint.y)) {
						if (map_(checkPoint.z, checkPoint.x, checkPoint.y) != g_config.passableTag) {
							return false;
						}
					}
				}
			}
		}
		return true;
	}

	double SingleDronePlanner::calculateStepWeight(Direction prevDir, Direction currDir) const {
		std::vector<int> horizontalDirs = { 0, 1, 2, 3, 4, 5, 6, 7 };

		if (prevDir == Direction::ORIGIN) {
			return g_config.weightStraight;
		}

		int prevVal = static_cast<int>(prevDir);
		int currVal = static_cast<int>(currDir);

		auto isHorizontal = [&](int val) {
			return std::find(horizontalDirs.begin(), horizontalDirs.end(), val) != horizontalDirs.end();
			};

		if (isHorizontal(prevVal)) {
			if (prevDir == currDir) {
				return g_config.weightStraight;
			}
			else if (isHorizontal(currVal)) {
				return g_config.weightHorizontal;
			}
			else if (currVal - prevVal == 8 || currVal - prevVal == 16) {
				return g_config.weightVertical;
			}
			else {
				return g_config.weightDiagonal;
			}
		}
		else if (prevVal >= 8 && prevVal <= 15) {
			if (prevVal - currVal == 8) {
				return g_config.weightStraight;
			}
			else if (isHorizontal(currVal)) {
				return g_config.weightHorizontal;
			}
			else if (prevVal == currVal) {
				return g_config.weightVertical;
			}
			else {
				return g_config.weightDiagonal;
			}
		}
		else {
			if (prevVal - currVal == 16) {
				return g_config.weightStraight;
			}
			else if (isHorizontal(currVal)) {
				return g_config.weightHorizontal;
			}
			else if (prevVal == currVal) {
				return g_config.weightVertical;
			}
			else {
				return g_config.weightDiagonal;
			}
		}
	}

	std::vector<std::pair<Point3D, Direction>> SingleDronePlanner::getNeighborDirections(const Point3D& point, Direction currentDir) const {
		std::vector<std::pair<Point3D, Direction>> neighbors;

		// 基础方向偏移
		int y1 = point.y - 1, y2 = point.y + 1;
		int x1 = point.x - 1, x2 = point.x + 1;
		int z1 = point.z - 1, z2 = point.z + 1;

		// 根据当前方向确定可能的邻居
		if (currentDir == Direction::ORIGIN) {
			neighbors = {
				{{point.x, y1, point.z}, Direction::S},
				{{point.x, y2, point.z}, Direction::N},
				{{x1, point.y, point.z}, Direction::W},
				{{x2, point.y, point.z}, Direction::E},
				{{x1, y1, point.z}, Direction::WS},
				{{x1, y2, point.z}, Direction::WN},
				{{x2, y1, point.z}, Direction::ES},
				{{x2, y2, point.z}, Direction::EN}
			};
		}
		else if (currentDir == Direction::N) {
			neighbors = {
				{{point.x, y2, point.z}, Direction::N},
				{{x1, y2, point.z}, Direction::WN},
				{{x2, y2, point.z}, Direction::EN},
				{{point.x, y2, z2}, Direction::UN},
				{{x1, y2, z2}, Direction::UWN},
				{{x2, y2, z2}, Direction::UEN},
				{{point.x, y2, z1}, Direction::DN},
				{{x1, y2, z1}, Direction::DWN},
				{{x2, y2, z1}, Direction::DEN}
			};
		}
		else if (currentDir == Direction::S) {
			neighbors = {
				{{point.x, y1, point.z}, Direction::S},
				{{x1, y1, point.z}, Direction::WS},
				{{x2, y1, point.z}, Direction::ES},
				{{point.x, y1, z2}, Direction::US},
				{{x1, y1, z2}, Direction::UWS},
				{{x2, y1, z2}, Direction::UES},
				{{point.x, y1, z1}, Direction::DS},
				{{x1, y1, z1}, Direction::DWS},
				{{x2, y1, z1}, Direction::DES}
			};
		}
		else if (currentDir == Direction::E) {
			neighbors = {
				{{x2, point.y, point.z}, Direction::E},
				{{x2, y1, point.z}, Direction::ES},
				{{x2, y2, point.z}, Direction::EN},
				{{x2, point.y, z2}, Direction::UE},
				{{x2, y1, z2}, Direction::UES},
				{{x2, y2, z2}, Direction::UEN},
				{{x2, point.y, z1}, Direction::DE},
				{{x2, y1, z1}, Direction::DES},
				{{x2, y2, z1}, Direction::DEN}
			};
		}
		else if (currentDir == Direction::W) {
			neighbors = {
				{{x1, point.y, point.z}, Direction::W},
				{{x1, y1, point.z}, Direction::WS},
				{{x1, y2, point.z}, Direction::WN},
				{{x1, point.y, z2}, Direction::UW},
				{{x1, y1, z2}, Direction::UWS},
				{{x1, y2, z2}, Direction::UWN},
				{{x1, point.y, z1}, Direction::DW},
				{{x1, y1, z1}, Direction::DWS},
				{{x1, y2, z1}, Direction::DWN}
			};
		}
		else {
			// 对于其他复杂方向，简化处理，返回基本的8个水平方向
			neighbors = {
				{{point.x, y1, point.z}, Direction::S},
				{{point.x, y2, point.z}, Direction::N},
				{{x1, point.y, point.z}, Direction::W},
				{{x2, point.y, point.z}, Direction::E},
				{{x1, y1, point.z}, Direction::WS},
				{{x1, y2, point.z}, Direction::WN},
				{{x2, y1, point.z}, Direction::ES},
				{{x2, y2, point.z}, Direction::EN}
			};
		}

		return neighbors;
	}

	void SingleDronePlanner::reset() {
		while (!openList_.empty()) {
			openList_.pop();
		}
		closedList_.clear();
		visitedNodes_.clear();
	}

} // namespace DronePathfinding