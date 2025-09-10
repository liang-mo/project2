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
			lastErrorMessage_ = "没有无人机需要规划路径";
			std::cout << lastErrorMessage_ << std::endl;
			return false;
		}

		std::cout << "开始为 " << drones_.size() << " 架无人机规划路径..." << std::endl;

		sortDronesByPriority();

		bool success = planWithPriorityBasedApproach();

		if (!success) {
			lastErrorMessage_ = "基于优先级的方法失败，尝试迭代方法";
			std::cout << lastErrorMessage_ << std::endl;
			success = planWithIterativeApproach();
		}

		auto endTime = std::chrono::high_resolution_clock::now();
		totalPlanningTime_ = std::chrono::duration<double>(endTime - startTime).count();

		if (success) {
			currentConflicts_ = ConflictDetector::detectConflicts(currentPaths_);
			std::cout << "路径规划完成，发现 " << currentConflicts_.size() << " 个冲突" << std::endl;
		}
		else {
			if (lastErrorMessage_.empty()) {
				lastErrorMessage_ = "路径规划失败，原因未知";
			}
			std::cout << lastErrorMessage_ << std::endl;
		}

		return success;
	}


	//bool MultiDronePlanner::planPaths() {
	//	auto startTime = std::chrono::high_resolution_clock::now();

	//	reset();

	//	if (drones_.empty()) {
	//		std::cout << "没有无人机需要规划路径" << std::endl;
	//		return false;
	//	}

	//	std::cout << "开始为 " << drones_.size() << " 架无人机规划路径..." << std::endl;

	//	// 按优先级排序无人机
	//	sortDronesByPriority();

	//	bool success = false;

	//	// 尝试基于优先级的方法
	//	success = planWithPriorityBasedApproach();

	//	if (!success) {
	//		std::cout << "基于优先级的方法失败，尝试迭代方法..." << std::endl;
	//		success = planWithIterativeApproach();
	//	}

	//	auto endTime = std::chrono::high_resolution_clock::now();
	//	totalPlanningTime_ = std::chrono::duration<double>(endTime - startTime).count();

	//	if (success) {
	//		currentConflicts_ = ConflictDetector::detectConflicts(currentPaths_);
	//		std::cout << "路径规划完成，发现 " << currentConflicts_.size() << " 个冲突" << std::endl;
	//	}
	//	else {
	//		std::cout << "路径规划失败" << std::endl;
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
				lastErrorMessage_ = "无人机 " + std::to_string(drone.id) + " 路径规划失败（优先级方法）";
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

	//		// 为当前无人机设置其他无人机的路径作为约束
	//		updateReservations(currentPaths_, drone.id);

	//		// 规划当前无人机的路径
	//		auto path = singlePlanner_->planPath(drone);

	//		if (path.empty()) {
	//			std::cout << "无人机 " << drone.id << " 路径规划失败" << std::endl;
	//			return false;
	//		}

	//		currentPaths_[i] = path;
	//		std::cout << "无人机 " << drone.id << " 路径规划成功，路径长度: " << path.size() << std::endl;
	//	}

	//	return true;
	//}

	bool MultiDronePlanner::planWithIterativeApproach() {
		currentPaths_.clear();
		currentPaths_.resize(drones_.size());

		// 初始规划（不考虑冲突）
		for (size_t i = 0; i < drones_.size(); i++) {
			singlePlanner_->clearReservedSpaceTime();
			auto path = singlePlanner_->planPath(drones_[i]);
			if (path.empty()) {
				std::cout << "无人机 " << drones_[i].id << " 初始路径规划失败" << std::endl;
				return false;
			}
			currentPaths_[i] = path;
		}

		// 迭代解决冲突
		for (iterationCount_ = 0; iterationCount_ < g_config.maxIterations; iterationCount_++) {
			auto conflicts = ConflictDetector::detectConflicts(currentPaths_);

			if (conflicts.empty()) {
				std::cout << "所有冲突已解决，迭代次数: " << iterationCount_ << std::endl;
				return true;
			}

			std::cout << "迭代 " << iterationCount_ << ": 发现 " << conflicts.size() << " 个冲突" << std::endl;

			// 选择第一个冲突进行解决
			const auto& conflict = conflicts[0];

			// 重新规划涉及冲突的无人机路径
			std::vector<int> conflictDrones = { conflict.droneId1, conflict.droneId2 };

			for (int droneId : conflictDrones) {
				// 找到无人机索引
				auto it = std::find_if(drones_.begin(), drones_.end(),
					[droneId](const DroneInfo& d) { return d.id == droneId; });

				if (it != drones_.end()) {
					size_t droneIndex = std::distance(drones_.begin(), it);

					// 设置约束（排除当前无人机）
					updateReservations(currentPaths_, droneId);

					// 重新规划路径
					auto newPath = singlePlanner_->planPath(*it);

					if (!newPath.empty()) {
						currentPaths_[droneIndex] = newPath;
						std::cout << "重新规划无人机 " << droneId << " 的路径" << std::endl;
					}
					else {
						std::cout << "无人机 " << droneId << " 重新规划失败" << std::endl;
					}
				}
			}
		}

		std::cout << "达到最大迭代次数，仍有冲突存在" << std::endl;
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
		// 检查每条路径的有效性
		for (size_t i = 0; i < paths.size(); i++) {
			const auto& path = paths[i];
			if (path.empty()) {
				return false;
			}

			// 检查起点和终点
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

				// 尝试获取原始坐标
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
				pointJson["Attribute"] = 2;  // 路径标记

				pathJson["points"].push_back(pointJson);
			}

			output["paths"].push_back(pathJson);
		}

		// 添加统计信息
		output["statistics"] = {
			{"total_drones", drones_.size()},
			{"total_conflicts", currentConflicts_.size()},
			{"iteration_count", iterationCount_},
			{"planning_time", totalPlanningTime_}
		};

		std::ofstream file(filename);
		file << std::setw(4) << output << std::endl;

		std::cout << "路径数据已保存到 " << filename << std::endl;
	}

	void MultiDronePlanner::printStatistics() const {
		std::cout << "\n=== 多无人机路径规划统计 ===" << std::endl;
		std::cout << "无人机数量: " << drones_.size() << std::endl;
		std::cout << "成功规划路径数: " << currentPaths_.size() << std::endl;
		std::cout << "总冲突数: " << currentConflicts_.size() << std::endl;
		std::cout << "迭代次数: " << iterationCount_ << std::endl;
		std::cout << "总规划时间: " << totalPlanningTime_ << " 秒" << std::endl;

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

	void MultiDronePlanner::reset() {
		currentPaths_.clear();
		currentConflicts_.clear();
		iterationCount_ = 0;
		totalNodesExplored_ = 0;
		totalPlanningTime_ = 0.0;
	}

} // namespace DronePathfinding