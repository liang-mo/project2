#include "multi_drone_planner.h"
#include "config.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <iomanip>


namespace DronePathfinding {

	MultiDronePlanner::MultiDronePlanner(const Map3D& map)
		: map_(map), iterationCount_(0), totalNodesExplored_(0), totalPlanningTime_(0.0) {
		singlePlanner_ = std::make_unique<SingleDronePlanner>(map);
	}

	void MultiDronePlanner::addDrone(const DroneInfo& drone) {
		drones_.push_back(drone);
	}

	void MultiDronePlanner::clearDrones() {
		drones_.clear();
		reset();
	}

	bool MultiDronePlanner::planPaths() {
		auto startTime = std::chrono::high_resolution_clock::now();
		reset();

		if (drones_.empty()) {
			lastErrorMessage_ = "û�����˻���Ҫ�滮·��";
			std::cout << lastErrorMessage_ << std::endl;
			return false;
		}

		std::cout << "��ʼΪ " << drones_.size() << " �����˻��滮·��..." << std::endl;

		sortDronesByPriority();

		bool success = planWithPriorityBasedApproach();

		if (!success) {
			lastErrorMessage_ = "�������ȼ��ķ���ʧ�ܣ����Ե�������";
			std::cout << lastErrorMessage_ << std::endl;
			success = planWithIterativeApproach();
		}

		auto endTime = std::chrono::high_resolution_clock::now();
		totalPlanningTime_ = std::chrono::duration<double>(endTime - startTime).count();

		if (success) {
			currentConflicts_ = ConflictDetector::detectConflicts(currentPaths_);
			std::cout << "·���滮��ɣ����� " << currentConflicts_.size() << " ����ͻ" << std::endl;
		}
		else {
			if (lastErrorMessage_.empty()) {
				lastErrorMessage_ = "·���滮ʧ�ܣ�ԭ��δ֪";
			}
			std::cout << lastErrorMessage_ << std::endl;
		}

		return success;
	}


	//bool MultiDronePlanner::planPaths() {
	//	auto startTime = std::chrono::high_resolution_clock::now();

	//	reset();

	//	if (drones_.empty()) {
	//		std::cout << "û�����˻���Ҫ�滮·��" << std::endl;
	//		return false;
	//	}

	//	std::cout << "��ʼΪ " << drones_.size() << " �����˻��滮·��..." << std::endl;

	//	// �����ȼ��������˻�
	//	sortDronesByPriority();

	//	bool success = false;

	//	// ���Ի������ȼ��ķ���
	//	success = planWithPriorityBasedApproach();

	//	if (!success) {
	//		std::cout << "�������ȼ��ķ���ʧ�ܣ����Ե�������..." << std::endl;
	//		success = planWithIterativeApproach();
	//	}

	//	auto endTime = std::chrono::high_resolution_clock::now();
	//	totalPlanningTime_ = std::chrono::duration<double>(endTime - startTime).count();

	//	if (success) {
	//		currentConflicts_ = ConflictDetector::detectConflicts(currentPaths_);
	//		std::cout << "·���滮��ɣ����� " << currentConflicts_.size() << " ����ͻ" << std::endl;
	//	}
	//	else {
	//		std::cout << "·���滮ʧ��" << std::endl;
	//	}

	//	return success;
	//}

	bool MultiDronePlanner::planWithPriorityBasedApproach() {
		currentPaths_.clear();
		currentPaths_.resize(drones_.size());

		for (size_t i = 0; i < drones_.size(); i++) {
			const auto& drone = drones_[i];
			updateReservations(currentPaths_, drone.id);

			auto path = singlePlanner_->planPath(drone);

			if (path.empty()) {
				lastErrorMessage_ = "���˻� " + std::to_string(drone.id) + " ·���滮ʧ�ܣ����ȼ�������";
				std::cout << lastErrorMessage_ << std::endl;
				return false;
			}

			currentPaths_[i] = path;
		}
		return true;
	}


	//bool MultiDronePlanner::planWithPriorityBasedApproach() {
	//	currentPaths_.clear();
	//	currentPaths_.resize(drones_.size());

	//	for (size_t i = 0; i < drones_.size(); i++) {
	//		const auto& drone = drones_[i];

	//		// Ϊ��ǰ���˻������������˻���·����ΪԼ��
	//		updateReservations(currentPaths_, drone.id);

	//		// �滮��ǰ���˻���·��
	//		auto path = singlePlanner_->planPath(drone);

	//		if (path.empty()) {
	//			std::cout << "���˻� " << drone.id << " ·���滮ʧ��" << std::endl;
	//			return false;
	//		}

	//		currentPaths_[i] = path;
	//		std::cout << "���˻� " << drone.id << " ·���滮�ɹ���·������: " << path.size() << std::endl;
	//	}

	//	return true;
	//}

	bool MultiDronePlanner::planWithIterativeApproach() {
		currentPaths_.clear();
		currentPaths_.resize(drones_.size());

		// ��ʼ�滮�������ǳ�ͻ��
		for (size_t i = 0; i < drones_.size(); i++) {
			singlePlanner_->clearReservedSpaceTime();
			auto path = singlePlanner_->planPath(drones_[i]);
			if (path.empty()) {
				std::cout << "���˻� " << drones_[i].id << " ��ʼ·���滮ʧ��" << std::endl;
				return false;
			}
			currentPaths_[i] = path;
		}

		// ���������ͻ
		for (iterationCount_ = 0; iterationCount_ < g_config.maxIterations; iterationCount_++) {
			auto conflicts = ConflictDetector::detectConflicts(currentPaths_);

			if (conflicts.empty()) {
				std::cout << "���г�ͻ�ѽ������������: " << iterationCount_ << std::endl;
				return true;
			}

			std::cout << "���� " << iterationCount_ << ": ���� " << conflicts.size() << " ����ͻ" << std::endl;

			// ѡ���һ����ͻ���н��
			const auto& conflict = conflicts[0];

			// ���¹滮�漰��ͻ�����˻�·��
			std::vector<int> conflictDrones = { conflict.droneId1, conflict.droneId2 };

			for (int droneId : conflictDrones) {
				// �ҵ����˻�����
				auto it = std::find_if(drones_.begin(), drones_.end(),
					[droneId](const DroneInfo& d) { return d.id == droneId; });

				if (it != drones_.end()) {
					size_t droneIndex = std::distance(drones_.begin(), it);

					// ����Լ�����ų���ǰ���˻���
					updateReservations(currentPaths_, droneId);

					// ���¹滮·��
					auto newPath = singlePlanner_->planPath(*it);

					if (!newPath.empty()) {
						currentPaths_[droneIndex] = newPath;
						std::cout << "���¹滮���˻� " << droneId << " ��·��" << std::endl;
					}
					else {
						std::cout << "���˻� " << droneId << " ���¹滮ʧ��" << std::endl;
					}
				}
			}
		}

		std::cout << "�ﵽ���������������г�ͻ����" << std::endl;
		return false;
	}

	void MultiDronePlanner::updateReservations(const Paths& paths, int excludeDroneId) {
		auto reservations = pathsToSpaceTimeReservations(paths, excludeDroneId);
		singlePlanner_->setReservedSpaceTime(reservations);
	}

	std::unordered_set<SpaceTimePoint, SpaceTimePointHash>
		MultiDronePlanner::pathsToSpaceTimeReservations(const Paths& paths, int excludeDroneId) const {
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

	void MultiDronePlanner::sortDronesByPriority() {
		std::sort(drones_.begin(), drones_.end(),
			[](const DroneInfo& a, const DroneInfo& b) {
				return a.priority > b.priority;
			});
	}

	bool MultiDronePlanner::validatePaths(const Paths& paths) const {
		// ���ÿ��·������Ч��
		for (size_t i = 0; i < paths.size(); i++) {
			const auto& path = paths[i];
			if (path.empty()) {
				return false;
			}

			// ��������յ�
			if (i < drones_.size()) {
				if (path.front()->point != drones_[i].startPoint ||
					path.back()->point != drones_[i].endPoint) {
					return false;
				}
			}
		}

		return true;
	}

	void MultiDronePlanner::savePathsToJson(const std::string& filename) const {
		nlohmann::json output;
		output["paths"] = nlohmann::json::array();

		for (size_t i = 0; i < currentPaths_.size(); i++) {
			nlohmann::json pathJson;
			pathJson["drone_id"] = (i < drones_.size()) ? drones_[i].id : static_cast<int>(i);
			pathJson["points"] = nlohmann::json::array();

			for (const auto& node : currentPaths_[i]) {
				nlohmann::json pointJson;

				// ���Ի�ȡԭʼ����
				if (map_.hasCoordMapping(node->point.z, node->point.x, node->point.y)) {
					auto coord = map_.getOriginalCoord(node->point.z, node->point.x, node->point.y);
					double origX = std::get<0>(coord);
					double origY = std::get<1>(coord);
					double origZ = std::get<2>(coord);
					pointJson["X"] = origX;
					pointJson["Y"] = origY;
					pointJson["Z"] = origZ;
				}
				else {
					pointJson["X"] = node->point.x;
					pointJson["Y"] = node->point.y;
					pointJson["Z"] = node->point.z;
				}

				pointJson["time_step"] = node->timeStep;
				pointJson["Attribute"] = 2;  // ·�����

				pathJson["points"].push_back(pointJson);
			}

			output["paths"].push_back(pathJson);
		}

		// ���ͳ����Ϣ
		output["statistics"] = {
			{"total_drones", drones_.size()},
			{"total_conflicts", currentConflicts_.size()},
			{"iteration_count", iterationCount_},
			{"planning_time", totalPlanningTime_}
		};

		std::ofstream file(filename);
		file << std::setw(4) << output << std::endl;

		std::cout << "·�������ѱ��浽 " << filename << std::endl;
	}

	void MultiDronePlanner::printStatistics() const {
		std::cout << "\n=== �����˻�·���滮ͳ�� ===" << std::endl;
		std::cout << "���˻�����: " << drones_.size() << std::endl;
		std::cout << "�ɹ��滮·����: " << currentPaths_.size() << std::endl;
		std::cout << "�ܳ�ͻ��: " << currentConflicts_.size() << std::endl;
		std::cout << "��������: " << iterationCount_ << std::endl;
		std::cout << "�ܹ滮ʱ��: " << totalPlanningTime_ << " ��" << std::endl;

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

	void MultiDronePlanner::reset() {
		currentPaths_.clear();
		currentConflicts_.clear();
		iterationCount_ = 0;
		totalNodesExplored_ = 0;
		totalPlanningTime_ = 0.0;
	}

} // namespace DronePathfinding