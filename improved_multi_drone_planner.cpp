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
			std::cout << "�Ľ��滮��: û�����˻���Ҫ�滮·��" << std::endl;
			return false;
		}

		std::cout << "�Ľ��滮��: ��ʼΪ " << drones_.size() << " �����˻��滮·��..." << std::endl;

		// �����ȼ��������˻�
		sortDronesByPriority();

		// ʹ�øĽ����Թ滮
		bool success = planWithImprovedStrategy();

		auto endTime = std::chrono::high_resolution_clock::now();
		totalPlanningTime_ = std::chrono::duration<double>(endTime - startTime).count();

		if (success) {
			// ����·������
			qualityMetrics_ = evaluatePathQuality(currentPaths_);
			currentConflicts_ = ConflictDetector::detectConflicts(currentPaths_);

			std::cout << "�Ľ��滮��: ·���滮���" << std::endl;
			std::cout << "  �ܳ�ͻ��: " << currentConflicts_.size() << std::endl;
			std::cout << "  ·����������: " << qualityMetrics_.getOverallScore() << std::endl;
			std::cout << "  �滮ʱ��: " << totalPlanningTime_ << " ��" << std::endl;
		}
		else {
			std::cout << "�Ľ��滮��: ·���滮ʧ��" << std::endl;
		}

		return success;
	}

	bool ImprovedMultiDronePlanner::planWithImprovedStrategy() {
		currentPaths_.clear();
		currentPaths_.resize(drones_.size());

		// ��һ�׶Σ���ʼ�滮����������Ӧ��ȫ���룩
		std::cout << "�׶�1: ��ʼ·���滮..." << std::endl;

		for (size_t i = 0; i < drones_.size(); i++) {
			const auto& drone = drones_[i];

			// �����������˻�·����ΪԼ��
			updateReservations(currentPaths_, drone.id);

			// �����������Ӧ��ȫ���룬��̬����
			if (useAdaptiveSafety_) {
				updateSafetyDistances();
			}

			// �滮·��
			auto path = singlePlanner_->planPath(drone);

			if (path.empty()) {
				std::cout << "���˻� " << drone.id << " ��ʼ�滮ʧ��" << std::endl;
				return false;
			}

			// ·��ƽ��
			if (usePathSmoothing_) {
				path = smoothPath(path);
			}

			// ��ȫ�Ż�
			path = optimizePathSafety(path, drone.id);

			currentPaths_[i] = path;
			std::cout << "���˻� " << drone.id << " ��ʼ·������: " << path.size() << std::endl;
		}

		// �ڶ��׶Σ����ܳ�ͻ���
		std::cout << "�׶�2: ���ܳ�ͻ���..." << std::endl;

		return resolveConflictsIntelligently();
	}

	bool ImprovedMultiDronePlanner::resolveConflictsIntelligently() {
		const int maxIterations = g_config.maxIterations;

		for (iterationCount_ = 0; iterationCount_ < maxIterations; iterationCount_++) {
			auto conflicts = ConflictDetector::detectConflicts(currentPaths_);

			if (conflicts.empty()) {
				std::cout << "���г�ͻ�ѽ������������: " << iterationCount_ << std::endl;
				return true;
			}

			std::cout << "���� " << iterationCount_ << ": ���� " << conflicts.size() << " ����ͻ" << std::endl;

			// ���ݲ��Խ����ͻ
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
				std::cout << "��ͻ���ʧ�ܣ����Խ�������..." << std::endl;
				// �������������ȼ�����
				resolved = resolvePriorityBased(conflicts);
			}

			if (!resolved) {
				std::cout << "�޷�������г�ͻ" << std::endl;
				break;
			}

			// ������ǰ����
			auto currentQuality = evaluatePathQuality(currentPaths_);
			if (currentQuality.getOverallScore() >= qualityThreshold_) {
				std::cout << "�ﵽ������ֵ����ǰ����" << std::endl;
				break;
			}
		}

		return iterationCount_ < maxIterations;
	}

	bool ImprovedMultiDronePlanner::resolvePriorityBased(const std::vector<Conflict>& conflicts) {
		// �����ȼ������ͻ
		for (const auto& conflict : conflicts) {
			// �ҵ���ͻ�����˻�
			auto drone1_it = std::find_if(drones_.begin(), drones_.end(),
				[&conflict](const DroneInfo& d) { return d.id == conflict.droneId1; });
			auto drone2_it = std::find_if(drones_.begin(), drones_.end(),
				[&conflict](const DroneInfo& d) { return d.id == conflict.droneId2; });

			if (drone1_it == drones_.end() || drone2_it == drones_.end()) {
				continue;
			}

			// ���¹滮���ȼ��ϵ͵����˻�
			int replanDroneId = (drone1_it->priority < drone2_it->priority) ?
				conflict.droneId1 : conflict.droneId2;

			auto replan_it = std::find_if(drones_.begin(), drones_.end(),
				[replanDroneId](const DroneInfo& d) { return d.id == replanDroneId; });

			if (replan_it != drones_.end()) {
				size_t droneIndex = std::distance(drones_.begin(), replan_it);

				// ����Լ��
				updateReservations(currentPaths_, replanDroneId);

				// ���¹滮
				auto newPath = singlePlanner_->planPath(*replan_it);

				if (!newPath.empty()) {
					if (usePathSmoothing_) {
						newPath = smoothPath(newPath);
					}
					newPath = optimizePathSafety(newPath, replanDroneId);
					currentPaths_[droneIndex] = newPath;
					std::cout << "���¹滮���˻� " << replanDroneId << " (���ȼ�����)" << std::endl;
				}
			}
		}

		return true;
	}

	bool ImprovedMultiDronePlanner::resolveTimeDelay(const std::vector<Conflict>& conflicts) {
		// ͨ��ʱ���ӳٽ����ͻ
		for (const auto& conflict : conflicts) {
			// Ϊ�ϵ����ȼ������˻�����ӳ�
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

				// ��·����ʼ��ӵȴ�ʱ��
				if (!path.empty()) {
					int delaySteps = 3; // �ӳ�3��ʱ�䲽
					Path delayedPath;

					// ��ӵȴ��ڵ�
					for (int i = 0; i < delaySteps; i++) {
						auto waitNode = std::make_shared<Node>(path[0]->point, delay_it->endPoint,
							delayDroneId, i);
						if (i > 0) {
							waitNode->parent = delayedPath.back();
						}
						delayedPath.push_back(waitNode);
					}

					// ���ԭ·��������ʱ�䲽��
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
					std::cout << "Ϊ���˻� " << delayDroneId << " ���ʱ���ӳ�" << std::endl;
				}
			}
		}

		return true;
	}

	bool ImprovedMultiDronePlanner::resolveSpatialSeparation(const std::vector<Conflict>& conflicts) {
		// ͨ�����ӿռ��������ͻ
		for (const auto& conflict : conflicts) {
			// ��ʱ���ӳ�ͻ����İ�ȫ����
			Point3D conflictLocation = conflict.location1;

			// Ϊ��ͻ������Χ�����ʱԼ��
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

			// ���¹滮�漰��ͻ�����˻�
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
						std::cout << "���¹滮���˻� " << droneId << " (�ռ�������)" << std::endl;
					}
				}
			}
		}

		return true;
	}

	bool ImprovedMultiDronePlanner::resolveAdaptiveSafety(const std::vector<Conflict>& conflicts) {
		// ��̬������ȫ��������ͻ
		std::unordered_map<int, int> droneConflictCount;

		// ͳ��ÿ�����˻��ĳ�ͻ����
		for (const auto& conflict : conflicts) {
			droneConflictCount[conflict.droneId1]++;
			droneConflictCount[conflict.droneId2]++;
		}

		// Ϊ��ͻ������˻����Ӱ�ȫ����
		for (auto it = droneConflictCount.begin(); it != droneConflictCount.end(); ++it) {
			int droneId = it->first;
			int conflictCount = it->second;
			if (conflictCount > 2) { // ��ͻ�϶�����˻�

				auto drone_it = std::find_if(drones_.begin(), drones_.end(),
					[droneId](const DroneInfo& d) { return d.id == droneId; });

				if (drone_it != drones_.end()) {
					size_t droneIndex = std::distance(drones_.begin(), drone_it);

					// ��ʱ���Ӱ�ȫ����
					int originalSafety = g_config.preferredSafetyDistance;
					g_config.preferredSafetyDistance = std::min(originalSafety + 2, 4);

					updateReservations(currentPaths_, droneId);
					auto newPath = singlePlanner_->planPath(*drone_it);

					// �ָ�ԭʼ��ȫ����
					g_config.preferredSafetyDistance = originalSafety;

					if (!newPath.empty()) {
						if (usePathSmoothing_) {
							newPath = smoothPath(newPath);
						}
						currentPaths_[droneIndex] = newPath;
						std::cout << "���¹滮���˻� " << droneId << " (����Ӧ��ȫ����)" << std::endl;
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

			// �ҵ���ֱ�ӵ������Զ��
			for (size_t i = current + 2; i < originalPath.size(); i++) {
				if (isDirectPathClear(originalPath[current]->point, originalPath[i]->point)) {
					farthest = i;
				}
				else {
					break;
				}
			}

			// ����м�ؼ����Ա���·��������
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
		// ��·�����а�ȫ�Ż����ܿ��߳�ͻ����
		Path optimizedPath = originalPath;

		if (useConflictPrediction_) {
			auto conflictZones = predictConflictZones();

			// ���·���Ƿ񾭹��߳�ͻ����
			for (size_t i = 0; i < optimizedPath.size(); i++) {
				const auto& point = optimizedPath[i]->point;

				if (isHighConflictZone(point)) {
					// �����ҵ����·����
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

							// �����½ڵ��滻ԭ�ڵ�
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

		// ���ڵ�ǰ·��Ԥ����ܵĳ�ͻ����
		std::unordered_map<Point3D, int, Point3DHash> pointUsage;

		for (const auto& path : currentPaths_) {
			for (const auto& node : path) {
				pointUsage[node->point]++;
			}
		}

		// ʹ��Ƶ�ʸߵĵ���Ϊ��ͻ����
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

			if (distance <= 2.0) { // �ڳ�ͻ����2��Χ��
				return true;
			}
		}

		return false;
	}

	PathQualityMetrics ImprovedMultiDronePlanner::evaluatePathQuality(const Paths& paths) const {
		PathQualityMetrics metrics;

		// �����ܳ���
		metrics.totalLength = 0.0;
		for (const auto& path : paths) {
			if (!path.empty()) {
				metrics.totalLength += path.back()->gCost;
			}
		}

		// ����ƽ����ȫ��
		metrics.averageSafety = calculateAverageSafety(paths);

		// �����ͻ����
		auto conflicts = ConflictDetector::detectConflicts(paths);
		metrics.conflictCount = static_cast<int>(conflicts.size());

		// ����ƽ����
		double totalSmoothness = 0.0;
		for (const auto& path : paths) {
			totalSmoothness += calculatePathSmoothness(path);
		}
		metrics.smoothness = paths.empty() ? 0.0 : totalSmoothness / paths.size();

		// ����Ч��
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

			// ���㷽��仯�Ƕ�
			Point3D vec1 = { curr.x - prev.x, curr.y - prev.y, curr.z - prev.z };
			Point3D vec2 = { next.x - curr.x, next.y - curr.y, next.z - curr.z };

			double dot = vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
			double mag1 = std::sqrt(vec1.x * vec1.x + vec1.y * vec1.y + vec1.z * vec1.z);
			double mag2 = std::sqrt(vec2.x * vec2.x + vec2.y * vec2.y + vec2.z * vec2.z);

			if (mag1 > 0 && mag2 > 0) {
				double cosAngle = dot / (mag1 * mag2);
				cosAngle = std::max(-1.0, std::min(1.0, cosAngle)); // ���Ʒ�Χ
				double angle = std::acos(cosAngle);
				totalAngleChange += angle;
				angleCount++;
			}
		}

		if (angleCount == 0) {
			return 1.0;
		}

		double avgAngleChange = totalAngleChange / angleCount;
		return std::max(0.0, 1.0 - avgAngleChange / M_PI); // ��һ����[0,1]
#include <cmath> // ��Ӵ�ͷ�ļ��Զ��� M_PI
	}

	double ImprovedMultiDronePlanner::calculatePathEfficiency(const Path& path, const DroneInfo& drone) const {
		if (path.empty()) {
			return 0.0;
		}

		// ����ֱ�߾���
		double directDistance = std::sqrt(
			std::pow(drone.endPoint.x - drone.startPoint.x, 2) +
			std::pow(drone.endPoint.y - drone.startPoint.y, 2) +
			std::pow(drone.endPoint.z - drone.startPoint.z, 2)
		);

		// ����ʵ��·������
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
		// ���ݵ�ǰ������̬������ȫ����
		// �������ʵ�ָ����ӵ�����Ӧ�߼�
	}

	int ImprovedMultiDronePlanner::calculateAdaptiveSafetyDistance(const Point3D& point, int droneId) const {
		// ���ھֲ������ܶȼ�������Ӧ��ȫ����
		int baseDistance = g_config.preferredSafetyDistance;

		// �����Χ�ϰ����ܶ�
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

		// �����ϰ����ܶȵ�����ȫ����
		double density = static_cast<double>(obstacleCount) /
			(std::pow(2 * checkRadius + 1, 3));

		if (density > 0.3) {
			return std::max(baseDistance - 1, 0); // ���ܶ����򽵵Ͱ�ȫ����
		}
		else if (density < 0.1) {
			return baseDistance + 1; // ���ܶ��������Ӱ�ȫ����
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

		// �����ϸͳ����Ϣ
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

		std::cout << "�Ľ��滮��: ·�������ѱ��浽 " << filename << std::endl;
	}

	void ImprovedMultiDronePlanner::printStatistics() const {
		std::cout << "\n=== �Ľ������˻�·���滮ͳ�� ===" << std::endl;
		std::cout << "���˻�����: " << drones_.size() << std::endl;
		std::cout << "�ɹ��滮·����: " << currentPaths_.size() << std::endl;
		std::cout << "�ܳ�ͻ��: " << currentConflicts_.size() << std::endl;
		std::cout << "��������: " << iterationCount_ << std::endl;
		std::cout << "�ܹ滮ʱ��: " << totalPlanningTime_ << " ��" << std::endl;

		std::cout << "\n·������ָ��:" << std::endl;
		std::cout << "  ��������: " << qualityMetrics_.getOverallScore() << std::endl;
		std::cout << "  ��·������: " << qualityMetrics_.totalLength << std::endl;
		std::cout << "  ƽ����ȫ��: " << qualityMetrics_.averageSafety << std::endl;
		std::cout << "  ·��ƽ����: " << qualityMetrics_.smoothness << std::endl;
		std::cout << "  ·��Ч��: " << qualityMetrics_.efficiency << std::endl;

		// ��ӡÿ�����˻���·������
		for (size_t i = 0; i < currentPaths_.size() && i < drones_.size(); i++) {
			std::cout << "���˻� " << drones_[i].id << " ·������: " << currentPaths_[i].size() << std::endl;
		}

		// ��ӡ��ͻ����
		if (!currentConflicts_.empty()) {
			std::cout << "\n��ͻ����:" << std::endl;
			for (const auto& conflict : currentConflicts_) {
				std::cout << "  ʱ�� " << conflict.timeStep << ": ���˻� "
					<< conflict.droneId1 << " �� " << conflict.droneId2;
				switch (conflict.type) {
				case ConflictType::VERTEX:
					std::cout << " �����ͻ";
					break;
				case ConflictType::EDGE:
					std::cout << " �߳�ͻ";
					break;
				case ConflictType::FOLLOWING:
					std::cout << " �����ͻ";
					break;
				default:
					std::cout << " δ֪��ͻ";
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