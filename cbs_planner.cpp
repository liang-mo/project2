#include "cbs_planner.h"
#include "config.h"
#include <iostream>
#include <chrono>
#include <algorithm>
#include <fstream>
#include <iomanip>


namespace DronePathfinding {

	CBSPlanner::CBSPlanner(const Map3D& map)
		: map_(map),
		openList_([](const CBSNodePtr& a, const CBSNodePtr& b) { return *a > *b; }),
		highLevelExpansions_(0), lowLevelExpansions_(0), totalPlanningTime_(0.0),
		maxHighLevelNodes_(10000), maxLowLevelNodes_(100000), timeLimit_(300.0),
		useDisjointSplitting_(true), usePositiveConstraints_(false) {

		lowLevelPlanner_ = std::make_unique<SingleDronePlanner>(map);
	}

	void CBSPlanner::addDrone(const DroneInfo& drone) {
		drones_.push_back(drone);
	}

	void CBSPlanner::clearDrones() {
		drones_.clear();
		reset();
	}

	bool CBSPlanner::planPaths() {
		auto startTime = std::chrono::high_resolution_clock::now();

		reset();

		if (drones_.empty()) {
			std::cout << "CBS: 没有无人机需要规划路径" << std::endl;
			return false;
		}

		std::cout << "CBS: 开始为 " << drones_.size() << " 架无人机规划路径..." << std::endl;

		// 创建根节点
		auto rootNode = createRootNode();
		if (!rootNode || !solveLowLevel(rootNode)) {
			std::cout << "CBS: 无法为根节点找到初始解" << std::endl;
			return false;
		}

		openList_.push(rootNode);

		// CBS主循环
		while (!openList_.empty() && highLevelExpansions_ < maxHighLevelNodes_) {
			auto currentTime = std::chrono::high_resolution_clock::now();
			double elapsed = std::chrono::duration<double>(currentTime - startTime).count();

			if (elapsed > timeLimit_) {
				std::cout << "CBS: 达到时间限制" << std::endl;
				break;
			}

			auto currentNode = openList_.top();
			openList_.pop();

			highLevelExpansions_++;

			// 检查是否找到无冲突解
			if (currentNode->conflicts.empty()) {
				std::cout << "CBS: 找到无冲突解！" << std::endl;
				std::cout << "高层扩展次数: " << highLevelExpansions_ << std::endl;
				std::cout << "低层扩展次数: " << lowLevelExpansions_ << std::endl;

				closedList_.push_back(currentNode);

				auto endTime = std::chrono::high_resolution_clock::now();
				totalPlanningTime_ = std::chrono::duration<double>(endTime - startTime).count();

				return true;
			}

			// 扩展当前节点
			auto childNodes = expandNode(currentNode);

			for (auto& child : childNodes) {
				if (solveLowLevel(child)) {
					openList_.push(child);
				}
			}

			closedList_.push_back(currentNode);

			// 定期输出进度
			if (highLevelExpansions_ % 100 == 0) {
				std::cout << "CBS: 高层扩展 " << highLevelExpansions_
					<< ", 当前最佳代价: " << currentNode->cost
					<< ", 冲突数: " << currentNode->conflicts.size() << std::endl;
			}
		}

		auto endTime = std::chrono::high_resolution_clock::now();
		totalPlanningTime_ = std::chrono::duration<double>(endTime - startTime).count();

		std::cout << "CBS: 未找到无冲突解" << std::endl;
		std::cout << "高层扩展次数: " << highLevelExpansions_ << std::endl;
		std::cout << "低层扩展次数: " << lowLevelExpansions_ << std::endl;

		return false;
	}

	CBSNodePtr CBSPlanner::createRootNode() {
		auto rootNode = std::make_shared<CBSNode>();
		rootNode->solution.resize(drones_.size());
		return rootNode;
	}

	std::vector<CBSNodePtr> CBSPlanner::expandNode(CBSNodePtr node) {
		std::vector<CBSNodePtr> children;

		if (node->conflicts.empty()) {
			return children;
		}

		// 选择要解决的冲突
		auto conflict = selectConflict(node->conflicts);

		if (useDisjointSplitting_) {
			// 使用分离分割策略
			children = disjointSplitting(conflict, node);
		}
		else {
			// 标准CBS分割
			auto constraints = generateConstraints(conflict, usePositiveConstraints_);

			for (const auto& constraint : constraints) {
				auto child = std::make_shared<CBSNode>(*node);
				child->constraints.push_back(constraint);
				child->depth = node->depth + 1;
				children.push_back(child);
			}
		}

		return children;
	}

	bool CBSPlanner::solveLowLevel(CBSNodePtr node) {
		// 为每个无人机重新规划路径
		for (size_t i = 0; i < drones_.size(); i++) {
			const auto& drone = drones_[i];

			// 应用约束
			clearConstraints();
			std::vector<CBSConstraint> droneConstraints;
			for (const auto& constraint : node->constraints) {
				if (constraint.droneId == drone.id) {
					droneConstraints.push_back(constraint);
				}
			}
			applyConstraints(drone.id, droneConstraints);

			// 规划路径
			auto path = lowLevelPlanner_->planPath(drone);
			lowLevelExpansions_++;

			if (path.empty()) {
				return false;  // 无法找到路径
			}

			node->solution[i] = path;
		}

		// 计算代价和检测冲突
		node->calculateCost();
		node->conflicts = ConflictDetector::detectConflicts(node->solution);

		return true;
	}

	std::vector<CBSConstraint> CBSPlanner::generateConstraints(const Conflict& conflict, bool usePositive) {
		std::vector<CBSConstraint> constraints;

		switch (conflict.type) {
		case ConflictType::VERTEX:
			// 顶点冲突：禁止两个无人机在同一时间访问同一位置
			constraints.emplace_back(ConstraintType::VERTEX, conflict.droneId1,
				conflict.location1, conflict.timeStep);
			constraints.emplace_back(ConstraintType::VERTEX, conflict.droneId2,
				conflict.location1, conflict.timeStep);
			break;

		case ConflictType::EDGE:
			// 边冲突：禁止两个无人机交换位置
			constraints.emplace_back(ConstraintType::EDGE, conflict.droneId1,
				conflict.location1, conflict.location2, conflict.timeStep);
			constraints.emplace_back(ConstraintType::EDGE, conflict.droneId2,
				conflict.location2, conflict.location1, conflict.timeStep);
			break;

		case ConflictType::FOLLOWING:
			// 跟随冲突：增加安全距离约束
			constraints.emplace_back(ConstraintType::VERTEX, conflict.droneId1,
				conflict.location1, conflict.timeStep);
			constraints.emplace_back(ConstraintType::VERTEX, conflict.droneId2,
				conflict.location2, conflict.timeStep);
			break;

		default:
			break;
		}

		// 如果使用正约束，添加正约束
		if (usePositive && !constraints.empty()) {
			// 简化实现：为第一个无人机添加正约束
			constraints.clear();
			constraints.emplace_back(ConstraintType::POSITIVE, conflict.droneId1,
				conflict.location1, conflict.timeStep);
			constraints.emplace_back(ConstraintType::VERTEX, conflict.droneId2,
				conflict.location1, conflict.timeStep);
		}

		return constraints;
	}

	void CBSPlanner::applyConstraints(int droneId, const std::vector<CBSConstraint>& constraints) {
		// 这里需要修改SingleDronePlanner以支持CBS约束
		// 暂时使用简化实现
		for (const auto& constraint : constraints) {
			if (constraint.type == ConstraintType::VERTEX) {
				lowLevelPlanner_->addReservedSpaceTime(constraint.location, constraint.timeStep);
			}
		}
	}

	void CBSPlanner::clearConstraints() {
		lowLevelPlanner_->clearReservedSpaceTime();
	}

	Conflict CBSPlanner::selectConflict(const std::vector<Conflict>& conflicts) {
		if (conflicts.empty()) {
			return Conflict(ConflictType::NONE, -1, -1, -1, Point3D());
		}

		// 简单策略：选择最早发生的冲突
		auto earliestConflict = *std::min_element(conflicts.begin(), conflicts.end(),
			[](const Conflict& a, const Conflict& b) {
				return a.timeStep < b.timeStep;
			});

		return earliestConflict;
	}

	bool CBSPlanner::isCardinalConflict(const Conflict& conflict, const Paths& solution) {
		// 简化实现：检查冲突是否是关键的
		// 真实实现需要检查添加约束后路径长度是否增加
		return true;
	}

	std::vector<CBSNodePtr> CBSPlanner::disjointSplitting(const Conflict& conflict, CBSNodePtr parent) {
		// 分离分割的简化实现
		// 真实实现需要更复杂的逻辑
		auto constraints = generateConstraints(conflict, false);
		std::vector<CBSNodePtr> children;

		for (const auto& constraint : constraints) {
			auto child = std::make_shared<CBSNode>(*parent);
			child->constraints.push_back(constraint);
			child->depth = parent->depth + 1;
			children.push_back(child);
		}

		return children;
	}

	void CBSPlanner::addPositiveConstraints(CBSNodePtr node, const Conflict& conflict) {
		// 正约束的实现
		// 这需要更复杂的逻辑来确保路径必须经过特定位置
	}

	Paths CBSPlanner::getBestSolution() const {
		if (!closedList_.empty()) {
			// 返回最后一个（最好的）解
			return closedList_.back()->solution;
		}
		return {};
	}

	std::vector<Conflict> CBSPlanner::getCurrentConflicts() const {
		if (!closedList_.empty()) {
			return closedList_.back()->conflicts;
		}
		return {};
	}

	void CBSPlanner::savePathsToJson(const std::string& filename) const {
		auto solution = getBestSolution();
		if (solution.empty()) {
			std::cout << "CBS: 没有可保存的解" << std::endl;
			return;
		}

		nlohmann::json output;
		output["algorithm"] = "CBS";
		output["paths"] = nlohmann::json::array();

		for (size_t i = 0; i < solution.size(); i++) {
			nlohmann::json pathJson;
			pathJson["drone_id"] = (i < drones_.size()) ? drones_[i].id : static_cast<int>(i);
			pathJson["points"] = nlohmann::json::array();

			for (const auto& node : solution[i]) {
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

		// 添加CBS统计信息
		output["statistics"] = {
			{"algorithm", "CBS"},
			{"total_drones", drones_.size()},
			{"high_level_expansions", highLevelExpansions_},
			{"low_level_expansions", lowLevelExpansions_},
			{"planning_time", totalPlanningTime_},
			{"total_conflicts", getCurrentConflicts().size()}
		};

		std::ofstream file(filename);
		file << std::setw(4) << output << std::endl;

		std::cout << "CBS: 路径数据已保存到 " << filename << std::endl;
	}

	void CBSPlanner::printStatistics() const {
		std::cout << "\n=== CBS算法统计信息 ===" << std::endl;
		std::cout << "无人机数量: " << drones_.size() << std::endl;
		std::cout << "高层扩展次数: " << highLevelExpansions_ << std::endl;
		std::cout << "低层扩展次数: " << lowLevelExpansions_ << std::endl;
		std::cout << "总规划时间: " << totalPlanningTime_ << " 秒" << std::endl;

		auto solution = getBestSolution();
		if (!solution.empty()) {
			double totalCost = 0.0;
			for (size_t i = 0; i < solution.size(); i++) {
				if (!solution[i].empty()) {
					double pathCost = solution[i].back()->gCost;
					totalCost += pathCost;
					std::cout << "无人机 " << (i < drones_.size() ? drones_[i].id : static_cast<int>(i))
						<< " 路径长度: " << solution[i].size()
						<< ", 代价: " << pathCost << std::endl;
				}
			}
			std::cout << "总代价: " << totalCost << std::endl;

			auto conflicts = getCurrentConflicts();
			std::cout << "剩余冲突数: " << conflicts.size() << std::endl;
		}
	}

	bool CBSPlanner::isValidSolution(const Paths& solution) const {
		for (size_t i = 0; i < solution.size() && i < drones_.size(); i++) {
			const auto& path = solution[i];
			if (path.empty()) {
				return false;
			}

			if (path.front()->point != drones_[i].startPoint ||
				path.back()->point != drones_[i].endPoint) {
				return false;
			}
		}
		return true;
	}

	void CBSPlanner::reset() {
		while (!openList_.empty()) {
			openList_.pop();
		}
		closedList_.clear();
		highLevelExpansions_ = 0;
		lowLevelExpansions_ = 0;
		totalPlanningTime_ = 0.0;
	}

	double CBSPlanner::calculateHeuristic(CBSNodePtr node) {
		// 简化的启发式：冲突数量
		return static_cast<double>(node->conflicts.size());
	}

	void CBSPlanner::prioritizeConflicts(std::vector<Conflict>& conflicts, const Paths& solution) {
		// 按时间步排序冲突
		std::sort(conflicts.begin(), conflicts.end(),
			[](const Conflict& a, const Conflict& b) {
				return a.timeStep < b.timeStep;
			});
	}

	// CBSConstraintManager实现
	void CBSConstraintManager::addConstraint(const CBSConstraint& constraint) {
		if (constraint.droneId == droneId_) {
			constraints_.push_back(constraint);
		}
	}

	void CBSConstraintManager::clearConstraints() {
		constraints_.clear();
	}

	bool CBSConstraintManager::isConstraintViolated(const Point3D& location, int timeStep) const {
		for (const auto& constraint : constraints_) {
			if (constraint.type == ConstraintType::VERTEX &&
				constraint.location == location &&
				constraint.timeStep == timeStep) {
				return true;
			}
		}
		return false;
	}

	bool CBSConstraintManager::isConstraintViolated(const Point3D& from, const Point3D& to, int timeStep) const {
		for (const auto& constraint : constraints_) {
			if (constraint.type == ConstraintType::EDGE &&
				constraint.fromLocation == from &&
				constraint.location == to &&
				constraint.timeStep == timeStep) {
				return true;
			}
		}
		return false;
	}

	bool CBSConstraintManager::hasPositiveConstraint(const Point3D& location, int timeStep) const {
		for (const auto& constraint : constraints_) {
			if (constraint.type == ConstraintType::POSITIVE &&
				constraint.location == location &&
				constraint.timeStep == timeStep) {
				return true;
			}
		}
		return false;
	}

} // namespace DronePathfinding