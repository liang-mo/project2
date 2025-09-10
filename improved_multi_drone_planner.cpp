#include "improved_multi_drone_planner.h"
#include "config.h"
#include "node.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <random>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace DronePathfinding {

	ImprovedMultiDronePlanner::ImprovedMultiDronePlanner(const Map3D& map)
		: map_(map), iterationCount_(0), totalNodesExplored_(0), totalPlanningTime_(0.0),
		strategy_(ConflictResolutionStrategy::ADAPTIVE_SAFETY),
		useAdaptiveSafety_(true), usePathSmoothing_(true), useConflictPrediction_(true),
		qualityThreshold_(0.8) {

		singlePlanner_ = std::make_unique<SingleDronePlanner>(map);
	}

	void ImprovedMultiDronePlanner::addDrone(const DroneInfo& drone) {
		drones_.push_back(drone);
	}

	void ImprovedMultiDronePlanner::clearDrones() {
		drones_.clear();
		reset();
	}

	bool ImprovedMultiDronePlanner::planPaths() {
		auto startTime = std::chrono::high_resolution_clock::now();

		reset();

		if (drones_.empty()) {
			std::cout << "改进规划器: 没有无人机需要规划路径" << std::endl;
			return false;
		}

		std::cout << "改进规划器: 开始为 " << drones_.size() << " 架无人机规划路径..." << std::endl;

		// 按优先级排序无人机
		sortDronesByPriority();

		// 使用改进策略规划
		bool success = planWithImprovedStrategy();

		auto endTime = std::chrono::high_resolution_clock::now();
		totalPlanningTime_ = std::chrono::duration<double>(endTime - startTime).count();

		if (success) {
			// 评估路径质量
			qualityMetrics_ = evaluatePathQuality(currentPaths_);
			currentConflicts_ = ConflictDetector::detectConflicts(currentPaths_);

			std::cout << "改进规划器: 路径规划完成" << std::endl;
			std::cout << "  总冲突数: " << currentConflicts_.size() << std::endl;
			std::cout << "  路径质量评分: " << qualityMetrics_.getOverallScore() << std::endl;
			std::cout << "  规划时间: " << totalPlanningTime_ << " 秒" << std::endl;
		}
		else {
			std::cout << "改进规划器: 路径规划失败" << std::endl;
		}

		return success;
	}

	bool ImprovedMultiDronePlanner::planWithImprovedStrategy() {
		currentPaths_.clear();
		currentPaths_.resize(drones_.size());

		// 第一阶段：初始规划（考虑自适应安全距离）
		std::cout << "阶段1: 初始路径规划..." << std::endl;

		for (size_t i = 0; i < drones_.size(); i++) {
			const auto& drone = drones_[i];

			// 设置其他无人机路径作为约束
			updateReservations(currentPaths_, drone.id);

			// 如果启用自适应安全距离，动态调整
			if (useAdaptiveSafety_) {
				updateSafetyDistances();
			}

			// 规划路径
			auto path = singlePlanner_->planPath(drone);

			if (path.empty()) {
				std::cout << "无人机 " << drone.id << " 初始规划失败" << std::endl;
				return false;
			}

			// 路径平滑
			if (usePathSmoothing_) {
				path = smoothPath(path);
			}

			// 安全优化
			path = optimizePathSafety(path, drone.id);

			currentPaths_[i] = path;
			std::cout << "无人机 " << drone.id << " 初始路径长度: " << path.size() << std::endl;
		}

		// 第二阶段：智能冲突解决
		std::cout << "阶段2: 智能冲突解决..." << std::endl;

		return resolveConflictsIntelligently();
	}

	bool ImprovedMultiDronePlanner::resolveConflictsIntelligently() {
		const int maxIterations = g_config.maxIterations;

		for (iterationCount_ = 0; iterationCount_ < maxIterations; iterationCount_++) {
			auto conflicts = ConflictDetector::detectConflicts(currentPaths_);

			if (conflicts.empty()) {
				std::cout << "所有冲突已解决，迭代次数: " << iterationCount_ << std::endl;
				return true;
			}

			std::cout << "迭代 " << iterationCount_ << ": 发现 " << conflicts.size() << " 个冲突" << std::endl;

			// 根据策略解决冲突
			bool resolved = false;
			switch (strategy_) {
			case ConflictResolutionStrategy::PRIORITY_BASED:
				resolved = resolvePriorityBased(conflicts);
				break;
			case ConflictResolutionStrategy::TIME_DELAY:
				resolved = resolveTimeDelay(conflicts);
				break;
			case ConflictResolutionStrategy::SPATIAL_SEPARATION:
				resolved = resolveSpatialSeparation(conflicts);
				break;
			case ConflictResolutionStrategy::ADAPTIVE_SAFETY:
				resolved = resolveAdaptiveSafety(conflicts);
				break;
			}

			if (!resolved) {
				std::cout << "冲突解决失败，尝试降级策略..." << std::endl;
				// 降级到基础优先级策略
				resolved = resolvePriorityBased(conflicts);
			}

			if (!resolved) {
				std::cout << "无法解决所有冲突" << std::endl;
				break;
			}

			// 评估当前质量
			auto currentQuality = evaluatePathQuality(currentPaths_);
			if (currentQuality.getOverallScore() >= qualityThreshold_) {
				std::cout << "达到质量阈值，提前结束" << std::endl;
				break;
			}
		}

		return iterationCount_ < maxIterations;
	}

	bool ImprovedMultiDronePlanner::resolvePriorityBased(const std::vector<Conflict>& conflicts) {
		// 按优先级解决冲突
		for (const auto& conflict : conflicts) {
			// 找到冲突的无人机
			auto drone1_it = std::find_if(drones_.begin(), drones_.end(),
				[&conflict](const DroneInfo& d) { return d.id == conflict.droneId1; });
			auto drone2_it = std::find_if(drones_.begin(), drones_.end(),
				[&conflict](const DroneInfo& d) { return d.id == conflict.droneId2; });

			if (drone1_it == drones_.end() || drone2_it == drones_.end()) {
				continue;
			}

			// 重新规划优先级较低的无人机
			int replanDroneId = (drone1_it->priority < drone2_it->priority) ?
				conflict.droneId1 : conflict.droneId2;

			auto replan_it = std::find_if(drones_.begin(), drones_.end(),
				[replanDroneId](const DroneInfo& d) { return d.id == replanDroneId; });

			if (replan_it != drones_.end()) {
				size_t droneIndex = std::distance(drones_.begin(), replan_it);

				// 设置约束
				updateReservations(currentPaths_, replanDroneId);

				// 重新规划
				auto newPath = singlePlanner_->planPath(*replan_it);

				if (!newPath.empty()) {
					if (usePathSmoothing_) {
						newPath = smoothPath(newPath);
					}
					newPath = optimizePathSafety(newPath, replanDroneId);
					currentPaths_[droneIndex] = newPath;
					std::cout << "重新规划无人机 " << replanDroneId << " (优先级策略)" << std::endl;
				}
			}
		}

		return true;
	}

	bool ImprovedMultiDronePlanner::resolveTimeDelay(const std::vector<Conflict>& conflicts) {
		// 通过时间延迟解决冲突
		for (const auto& conflict : conflicts) {
			// 为较低优先级的无人机添加延迟
			auto drone1_it = std::find_if(drones_.begin(), drones_.end(),
				[&conflict](const DroneInfo& d) { return d.id == conflict.droneId1; });
			auto drone2_it = std::find_if(drones_.begin(), drones_.end(),
				[&conflict](const DroneInfo& d) { return d.id == conflict.droneId2; });

			if (drone1_it == drones_.end() || drone2_it == drones_.end()) {
				continue;
			}

			int delayDroneId = (drone1_it->priority < drone2_it->priority) ?
				conflict.droneId1 : conflict.droneId2;

			auto delay_it = std::find_if(drones_.begin(), drones_.end(),
				[delayDroneId](const DroneInfo& d) { return d.id == delayDroneId; });

			if (delay_it != drones_.end()) {
				size_t droneIndex = std::distance(drones_.begin(), delay_it);
				auto& path = currentPaths_[droneIndex];

				// 在路径开始添加等待时间
				if (!path.empty()) {
					int delaySteps = 3; // 延迟3个时间步
					Path delayedPath;

					// 添加等待节点
					for (int i = 0; i < delaySteps; i++) {
						auto waitNode = std::make_shared<Node>(path[0]->point, delay_it->endPoint,
							delayDroneId, i);
						if (i > 0) {
							waitNode->parent = delayedPath.back();
						}
						delayedPath.push_back(waitNode);
					}

					// 添加原路径（调整时间步）
					for (size_t i = 0; i < path.size(); i++) {
						auto newNode = std::make_shared<Node>(*path[i]);
						newNode->timeStep += delaySteps;
						if (i == 0) {
							newNode->parent = delayedPath.back();
						}
						else {
							newNode->parent = delayedPath[delayedPath.size() - 1];
						}
						delayedPath.push_back(newNode);
					}

					currentPaths_[droneIndex] = delayedPath;
					std::cout << "为无人机 " << delayDroneId << " 添加时间延迟" << std::endl;
				}
			}
		}

		return true;
	}

	bool ImprovedMultiDronePlanner::resolveSpatialSeparation(const std::vector<Conflict>& conflicts) {
		// 通过增加空间分离解决冲突
		for (const auto& conflict : conflicts) {
			// 临时增加冲突区域的安全距离
			Point3D conflictLocation = conflict.location1;

			// 为冲突区域周围添加临时约束
			for (int dx = -2; dx <= 2; dx++) {
				for (int dy = -2; dy <= 2; dy++) {
					for (int dz = -1; dz <= 1; dz++) {
						Point3D reservedPoint(conflictLocation.x + dx,
							conflictLocation.y + dy,
							conflictLocation.z + dz);
						singlePlanner_->addReservedSpaceTime(reservedPoint, conflict.timeStep);
					}
				}
			}

			// 重新规划涉及冲突的无人机
			std::vector<int> conflictDrones = { conflict.droneId1, conflict.droneId2 };

			for (int droneId : conflictDrones) {
				auto drone_it = std::find_if(drones_.begin(), drones_.end(),
					[droneId](const DroneInfo& d) { return d.id == droneId; });

				if (drone_it != drones_.end()) {
					size_t droneIndex = std::distance(drones_.begin(), drone_it);

					updateReservations(currentPaths_, droneId);
					auto newPath = singlePlanner_->planPath(*drone_it);

					if (!newPath.empty()) {
						if (usePathSmoothing_) {
							newPath = smoothPath(newPath);
						}
						currentPaths_[droneIndex] = newPath;
						std::cout << "重新规划无人机 " << droneId << " (空间分离策略)" << std::endl;
					}
				}
			}
		}

		return true;
	}

	bool ImprovedMultiDronePlanner::resolveAdaptiveSafety(const std::vector<Conflict>& conflicts) {
		// 动态调整安全距离解决冲突
		std::unordered_map<int, int> droneConflictCount;

		// 统计每个无人机的冲突次数
		for (const auto& conflict : conflicts) {
			droneConflictCount[conflict.droneId1]++;
			droneConflictCount[conflict.droneId2]++;
		}

		// 为冲突多的无人机增加安全距离
		for (auto it = droneConflictCount.begin(); it != droneConflictCount.end(); ++it) {
			int droneId = it->first;
			int conflictCount = it->second;
			if (conflictCount > 2) { // 冲突较多的无人机

				auto drone_it = std::find_if(drones_.begin(), drones_.end(),
					[droneId](const DroneInfo& d) { return d.id == droneId; });

				if (drone_it != drones_.end()) {
					size_t droneIndex = std::distance(drones_.begin(), drone_it);

					// 临时增加安全距离
					int originalSafety = g_config.preferredSafetyDistance;
					g_config.preferredSafetyDistance = std::min(originalSafety + 2, 4);

					updateReservations(currentPaths_, droneId);
					auto newPath = singlePlanner_->planPath(*drone_it);

					// 恢复原始安全距离
					g_config.preferredSafetyDistance = originalSafety;

					if (!newPath.empty()) {
						if (usePathSmoothing_) {
							newPath = smoothPath(newPath);
						}
						currentPaths_[droneIndex] = newPath;
						std::cout << "重新规划无人机 " << droneId << " (自适应安全距离)" << std::endl;
					}
				}
			}
		}

		return true;
	}

	Path ImprovedMultiDronePlanner::smoothPath(const Path& originalPath) const {
		if (originalPath.size() <= 2) {
			return originalPath;
		}

		Path smoothedPath;
		smoothedPath.push_back(originalPath[0]);

		size_t current = 0;
		while (current < originalPath.size() - 1) {
			size_t farthest = current + 1;

			// 找到能直接到达的最远点
			for (size_t i = current + 2; i < originalPath.size(); i++) {
				if (isDirectPathClear(originalPath[current]->point, originalPath[i]->point)) {
					farthest = i;
				}
				else {
					break;
				}
			}

			// 添加中间关键点以保持路径连续性
			if (farthest > current + 2) {
				size_t mid = (current + farthest) / 2;
				smoothedPath.push_back(originalPath[mid]);
			}

			smoothedPath.push_back(originalPath[farthest]);
			current = farthest;
		}

		return smoothedPath;
	}


	bool ImprovedMultiDronePlanner::isDirectPathClear(const Point3D& start, const Point3D& end) const {
		int dx = std::abs(end.x - start.x);
		int dy = std::abs(end.y - start.y);
		int dz = std::abs(end.z - start.z);

		int steps = std::max({ dx, dy, dz });
		if (steps == 0) return true;

		for (int i = 0; i <= steps; i++) {
			Point3D checkPoint(
				start.x + (end.x - start.x) * i / steps,
				start.y + (end.y - start.y) * i / steps,
				start.z + (end.z - start.z) * i / steps
			);

			if (!map_.isValidIndex(checkPoint.z, checkPoint.x, checkPoint.y) ||
				map_(checkPoint.z, checkPoint.x, checkPoint.y) != 0) {
				return false;
			}
		}

		return true;
	}

	Path ImprovedMultiDronePlanner::optimizePathSafety(const Path& originalPath, int droneId) const {
		// 对路径进行安全优化，避开高冲突区域
		Path optimizedPath = originalPath;

		if (useConflictPrediction_) {
			auto conflictZones = predictConflictZones();

			// 检查路径是否经过高冲突区域
			for (size_t i = 0; i < optimizedPath.size(); i++) {
				const auto& point = optimizedPath[i]->point;

				if (isHighConflictZone(point)) {
					// 尝试找到替代路径点
					std::vector<Point3D> alternatives = {
						{point.x + 1, point.y, point.z},
						{point.x - 1, point.y, point.z},
						{point.x, point.y + 1, point.z},
						{point.x, point.y - 1, point.z},
						{point.x, point.y, point.z + 1},
						{point.x, point.y, point.z - 1}
					};

					for (const auto& alt : alternatives) {
						if (map_.isValidIndex(alt.z, alt.x, alt.y) &&
							map_(alt.z, alt.x, alt.y) == 0 &&
							!isHighConflictZone(alt)) {

							// 创建新节点替换原节点
							auto newNode = std::make_shared<Node>(*optimizedPath[i]);
							newNode->point = alt;
							optimizedPath[i] = newNode;
							break;
						}
					}
				}
			}
		}

		return optimizedPath;
	}

	std::vector<Point3D> ImprovedMultiDronePlanner::predictConflictZones() const {
		std::vector<Point3D> conflictZones;

		// 基于当前路径预测可能的冲突区域
		std::unordered_map<Point3D, int, Point3DHash> pointUsage;

		for (const auto& path : currentPaths_) {
			for (const auto& node : path) {
				pointUsage[node->point]++;
			}
		}

		// 使用频率高的点视为冲突区域
		for (auto it = pointUsage.begin(); it != pointUsage.end(); ++it) {
			const Point3D& point = it->first;
			int usage = it->second;
			if (usage > 1) {
				conflictZones.push_back(point);
			}
		}

		return conflictZones;
	}

	bool ImprovedMultiDronePlanner::isHighConflictZone(const Point3D& point) const {
		auto conflictZones = predictConflictZones();

		for (const auto& zone : conflictZones) {
			double distance = std::sqrt(
				std::pow(point.x - zone.x, 2) +
				std::pow(point.y - zone.y, 2) +
				std::pow(point.z - zone.z, 2)
			);

			if (distance <= 2.0) { // 在冲突区域2格范围内
				return true;
			}
		}

		return false;
	}

	PathQualityMetrics ImprovedMultiDronePlanner::evaluatePathQuality(const Paths& paths) const {
		PathQualityMetrics metrics;

		// 计算总长度
		metrics.totalLength = 0.0;
		for (const auto& path : paths) {
			if (!path.empty()) {
				metrics.totalLength += path.back()->gCost;
			}
		}

		// 计算平均安全性
		metrics.averageSafety = calculateAverageSafety(paths);

		// 计算冲突数量
		auto conflicts = ConflictDetector::detectConflicts(paths);
		metrics.conflictCount = static_cast<int>(conflicts.size());

		// 计算平滑度
		double totalSmoothness = 0.0;
		for (const auto& path : paths) {
			totalSmoothness += calculatePathSmoothness(path);
		}
		metrics.smoothness = paths.empty() ? 0.0 : totalSmoothness / paths.size();

		// 计算效率
		double totalEfficiency = 0.0;
		for (size_t i = 0; i < paths.size() && i < drones_.size(); i++) {
			totalEfficiency += calculatePathEfficiency(paths[i], drones_[i]);
		}
		metrics.efficiency = paths.empty() ? 0.0 : totalEfficiency / paths.size();

		return metrics;
	}

	double ImprovedMultiDronePlanner::calculatePathSmoothness(const Path& path) const {
		if (path.size() < 3) {
			return 1.0;
		}

		double totalAngleChange = 0.0;
		int angleCount = 0;

		for (size_t i = 1; i < path.size() - 1; i++) {
			Point3D prev = path[i - 1]->point;
			Point3D curr = path[i]->point;
			Point3D next = path[i + 1]->point;

			// 计算方向变化角度
			Point3D vec1 = { curr.x - prev.x, curr.y - prev.y, curr.z - prev.z };
			Point3D vec2 = { next.x - curr.x, next.y - curr.y, next.z - curr.z };

			double dot = vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
			double mag1 = std::sqrt(vec1.x * vec1.x + vec1.y * vec1.y + vec1.z * vec1.z);
			double mag2 = std::sqrt(vec2.x * vec2.x + vec2.y * vec2.y + vec2.z * vec2.z);

			if (mag1 > 0 && mag2 > 0) {
				double cosAngle = dot / (mag1 * mag2);
				cosAngle = std::max(-1.0, std::min(1.0, cosAngle)); // 限制范围
				double angle = std::acos(cosAngle);
				totalAngleChange += angle;
				angleCount++;
			}
		}

		if (angleCount == 0) {
			return 1.0;
		}

		double avgAngleChange = totalAngleChange / angleCount;
		return std::max(0.0, 1.0 - avgAngleChange / M_PI); // 归一化到[0,1]
#include <cmath> // 添加此头文件以定义 M_PI
	}

	double ImprovedMultiDronePlanner::calculatePathEfficiency(const Path& path, const DroneInfo& drone) const {
		if (path.empty()) {
			return 0.0;
		}

		// 计算直线距离
		double directDistance = std::sqrt(
			std::pow(drone.endPoint.x - drone.startPoint.x, 2) +
			std::pow(drone.endPoint.y - drone.startPoint.y, 2) +
			std::pow(drone.endPoint.z - drone.startPoint.z, 2)
		);

		// 计算实际路径长度
		double actualDistance = path.back()->gCost;

		if (actualDistance == 0.0) {
			return 1.0;
		}

		return directDistance / actualDistance;
	}

	double ImprovedMultiDronePlanner::calculateAverageSafety(const Paths& paths) const {
		if (paths.empty()) {
			return 0.0;
		}

		double totalSafety = 0.0;
		int safetyCount = 0;

		for (const auto& path : paths) {
			for (const auto& node : path) {
				int safetyLevel = singlePlanner_->calculateSafetyLevel(node->point);
				totalSafety += (g_config.preferredSafetyDistance - safetyLevel) /
					static_cast<double>(g_config.preferredSafetyDistance);
				safetyCount++;
			}
		}

		return safetyCount > 0 ? totalSafety / safetyCount : 0.0;
	}

	void ImprovedMultiDronePlanner::updateSafetyDistances() {
		// 根据当前环境动态调整安全距离
		// 这里可以实现更复杂的自适应逻辑
	}

	int ImprovedMultiDronePlanner::calculateAdaptiveSafetyDistance(const Point3D& point, int droneId) const {
		// 基于局部环境密度计算自适应安全距离
		int baseDistance = g_config.preferredSafetyDistance;

		// 检查周围障碍物密度
		int obstacleCount = 0;
		int checkRadius = 3;

		for (int dx = -checkRadius; dx <= checkRadius; dx++) {
			for (int dy = -checkRadius; dy <= checkRadius; dy++) {
				for (int dz = -checkRadius; dz <= checkRadius; dz++) {
					Point3D checkPoint(point.x + dx, point.y + dy, point.z + dz);
					if (map_.isValidIndex(checkPoint.z, checkPoint.x, checkPoint.y) &&
						map_(checkPoint.z, checkPoint.x, checkPoint.y) != 0) {
						obstacleCount++;
					}
				}
			}
		}

		// 根据障碍物密度调整安全距离
		double density = static_cast<double>(obstacleCount) /
			(std::pow(2 * checkRadius + 1, 3));

		if (density > 0.3) {
			return std::max(baseDistance - 1, 0); // 高密度区域降低安全距离
		}
		else if (density < 0.1) {
			return baseDistance + 1; // 低密度区域增加安全距离
		}

		return baseDistance;
	}

	void ImprovedMultiDronePlanner::sortDronesByPriority() {
		std::sort(drones_.begin(), drones_.end(),
			[](const DroneInfo& a, const DroneInfo& b) {
				return a.priority > b.priority;
			});
	}

	void ImprovedMultiDronePlanner::updateReservations(const Paths& paths, int excludeDroneId) {
		auto reservations = pathsToSpaceTimeReservations(paths, excludeDroneId);
		singlePlanner_->setReservedSpaceTime(reservations);
	}

	std::unordered_set<SpaceTimePoint, SpaceTimePointHash>
		ImprovedMultiDronePlanner::pathsToSpaceTimeReservations(const Paths& paths, int excludeDroneId) const {
		std::unordered_set<SpaceTimePoint, SpaceTimePointHash> reservations;

		for (size_t i = 0; i < paths.size(); i++) {
			if (i < drones_.size() && drones_[i].id == excludeDroneId) {
				continue;
			}

			const auto& path = paths[i];
			for (size_t t = 0; t < path.size(); t++) {
				reservations.emplace(path[t]->point, static_cast<int>(t));
			}
		}

		return reservations;
	}

	bool ImprovedMultiDronePlanner::validatePaths(const Paths& paths) const {
		for (size_t i = 0; i < paths.size(); i++) {
			const auto& path = paths[i];
			if (path.empty()) {
				return false;
			}

			if (i < drones_.size()) {
				if (path.front()->point != drones_[i].startPoint ||
					path.back()->point != drones_[i].endPoint) {
					return false;
				}
			}
		}

		return true;
	}

	void ImprovedMultiDronePlanner::savePathsToJson(const std::string& filename) const {
		nlohmann::json output;
		output["algorithm"] = "ImprovedMultiDrone";
		output["paths"] = nlohmann::json::array();

		for (size_t i = 0; i < currentPaths_.size(); i++) {
			nlohmann::json pathJson;
			pathJson["drone_id"] = (i < drones_.size()) ? drones_[i].id : static_cast<int>(i);
			pathJson["points"] = nlohmann::json::array();

			for (const auto& node : currentPaths_[i]) {
				nlohmann::json pointJson;

				if (map_.hasCoordMapping(node->point.z, node->point.x, node->point.y)) {
					auto coord = map_.getOriginalCoord(node->point.z, node->point.x, node->point.y);
					pointJson["X"] = std::get<0>(coord);
					pointJson["Y"] = std::get<1>(coord);
					pointJson["Z"] = std::get<2>(coord);
				}
				else {
					pointJson["X"] = node->point.x;
					pointJson["Y"] = node->point.y;
					pointJson["Z"] = node->point.z;
				}

				pointJson["time_step"] = node->timeStep;
				pointJson["Attribute"] = 2;

				pathJson["points"].push_back(pointJson);
			}

			output["paths"].push_back(pathJson);
		}

		// 添加详细统计信息
		output["statistics"] = {
			{"algorithm", "ImprovedMultiDrone"},
			{"total_drones", drones_.size()},
			{"total_conflicts", currentConflicts_.size()},
			{"iteration_count", iterationCount_},
			{"planning_time", totalPlanningTime_},
			{"quality_score", qualityMetrics_.getOverallScore()},
			{"total_length", qualityMetrics_.totalLength},
			{"average_safety", qualityMetrics_.averageSafety},
			{"smoothness", qualityMetrics_.smoothness},
			{"efficiency", qualityMetrics_.efficiency}
		};

		std::ofstream file(filename);
		file << std::setw(4) << output << std::endl;

		std::cout << "改进规划器: 路径数据已保存到 " << filename << std::endl;
	}

	void ImprovedMultiDronePlanner::printStatistics() const {
		std::cout << "\n=== 改进多无人机路径规划统计 ===" << std::endl;
		std::cout << "无人机数量: " << drones_.size() << std::endl;
		std::cout << "成功规划路径数: " << currentPaths_.size() << std::endl;
		std::cout << "总冲突数: " << currentConflicts_.size() << std::endl;
		std::cout << "迭代次数: " << iterationCount_ << std::endl;
		std::cout << "总规划时间: " << totalPlanningTime_ << " 秒" << std::endl;

		std::cout << "\n路径质量指标:" << std::endl;
		std::cout << "  总体评分: " << qualityMetrics_.getOverallScore() << std::endl;
		std::cout << "  总路径长度: " << qualityMetrics_.totalLength << std::endl;
		std::cout << "  平均安全性: " << qualityMetrics_.averageSafety << std::endl;
		std::cout << "  路径平滑度: " << qualityMetrics_.smoothness << std::endl;
		std::cout << "  路径效率: " << qualityMetrics_.efficiency << std::endl;

		// 打印每架无人机的路径长度
		for (size_t i = 0; i < currentPaths_.size() && i < drones_.size(); i++) {
			std::cout << "无人机 " << drones_[i].id << " 路径长度: " << currentPaths_[i].size() << std::endl;
		}

		// 打印冲突详情
		if (!currentConflicts_.empty()) {
			std::cout << "\n冲突详情:" << std::endl;
			for (const auto& conflict : currentConflicts_) {
				std::cout << "  时间 " << conflict.timeStep << ": 无人机 "
					<< conflict.droneId1 << " 与 " << conflict.droneId2;
				switch (conflict.type) {
				case ConflictType::VERTEX:
					std::cout << " 顶点冲突";
					break;
				case ConflictType::EDGE:
					std::cout << " 边冲突";
					break;
				case ConflictType::FOLLOWING:
					std::cout << " 跟随冲突";
					break;
				default:
					std::cout << " 未知冲突";
				}
				std::cout << std::endl;
			}
		}
	}

	void ImprovedMultiDronePlanner::reset() {
		currentPaths_.clear();
		currentConflicts_.clear();
		iterationCount_ = 0;
		totalNodesExplored_ = 0;
		totalPlanningTime_ = 0.0;
		qualityMetrics_ = PathQualityMetrics();
	}

} // namespace DronePathfinding