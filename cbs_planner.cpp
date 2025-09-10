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
			std::cout << "CBS: û�����˻���Ҫ�滮·��" << std::endl;
			return false;
		}

		std::cout << "CBS: ��ʼΪ " << drones_.size() << " �����˻��滮·��..." << std::endl;

		// �������ڵ�
		auto rootNode = createRootNode();
		if (!rootNode || !solveLowLevel(rootNode)) {
			std::cout << "CBS: �޷�Ϊ���ڵ��ҵ���ʼ��" << std::endl;
			return false;
		}

		openList_.push(rootNode);

		// CBS��ѭ��
		while (!openList_.empty() && highLevelExpansions_ < maxHighLevelNodes_) {
			auto currentTime = std::chrono::high_resolution_clock::now();
			double elapsed = std::chrono::duration<double>(currentTime - startTime).count();

			if (elapsed > timeLimit_) {
				std::cout << "CBS: �ﵽʱ������" << std::endl;
				break;
			}

			auto currentNode = openList_.top();
			openList_.pop();

			highLevelExpansions_++;

			// ����Ƿ��ҵ��޳�ͻ��
			if (currentNode->conflicts.empty()) {
				std::cout << "CBS: �ҵ��޳�ͻ�⣡" << std::endl;
				std::cout << "�߲���չ����: " << highLevelExpansions_ << std::endl;
				std::cout << "�Ͳ���չ����: " << lowLevelExpansions_ << std::endl;

				closedList_.push_back(currentNode);

				auto endTime = std::chrono::high_resolution_clock::now();
				totalPlanningTime_ = std::chrono::duration<double>(endTime - startTime).count();

				return true;
			}

			// ��չ��ǰ�ڵ�
			auto childNodes = expandNode(currentNode);

			for (auto& child : childNodes) {
				if (solveLowLevel(child)) {
					openList_.push(child);
				}
			}

			closedList_.push_back(currentNode);

			// �����������
			if (highLevelExpansions_ % 100 == 0) {
				std::cout << "CBS: �߲���չ " << highLevelExpansions_
					<< ", ��ǰ��Ѵ���: " << currentNode->cost
					<< ", ��ͻ��: " << currentNode->conflicts.size() << std::endl;
			}
		}

		auto endTime = std::chrono::high_resolution_clock::now();
		totalPlanningTime_ = std::chrono::duration<double>(endTime - startTime).count();

		std::cout << "CBS: δ�ҵ��޳�ͻ��" << std::endl;
		std::cout << "�߲���չ����: " << highLevelExpansions_ << std::endl;
		std::cout << "�Ͳ���չ����: " << lowLevelExpansions_ << std::endl;

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

		// ѡ��Ҫ����ĳ�ͻ
		auto conflict = selectConflict(node->conflicts);

		if (useDisjointSplitting_) {
			// ʹ�÷���ָ����
			children = disjointSplitting(conflict, node);
		}
		else {
			// ��׼CBS�ָ�
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
		// Ϊÿ�����˻����¹滮·��
		for (size_t i = 0; i < drones_.size(); i++) {
			const auto& drone = drones_[i];

			// Ӧ��Լ��
			clearConstraints();
			std::vector<CBSConstraint> droneConstraints;
			for (const auto& constraint : node->constraints) {
				if (constraint.droneId == drone.id) {
					droneConstraints.push_back(constraint);
				}
			}
			applyConstraints(drone.id, droneConstraints);

			// �滮·��
			auto path = lowLevelPlanner_->planPath(drone);
			lowLevelExpansions_++;

			if (path.empty()) {
				return false;  // �޷��ҵ�·��
			}

			node->solution[i] = path;
		}

		// ������ۺͼ���ͻ
		node->calculateCost();
		node->conflicts = ConflictDetector::detectConflicts(node->solution);

		return true;
	}

	std::vector<CBSConstraint> CBSPlanner::generateConstraints(const Conflict& conflict, bool usePositive) {
		std::vector<CBSConstraint> constraints;

		switch (conflict.type) {
		case ConflictType::VERTEX:
			// �����ͻ����ֹ�������˻���ͬһʱ�����ͬһλ��
			constraints.emplace_back(ConstraintType::VERTEX, conflict.droneId1,
				conflict.location1, conflict.timeStep);
			constraints.emplace_back(ConstraintType::VERTEX, conflict.droneId2,
				conflict.location1, conflict.timeStep);
			break;

		case ConflictType::EDGE:
			// �߳�ͻ����ֹ�������˻�����λ��
			constraints.emplace_back(ConstraintType::EDGE, conflict.droneId1,
				conflict.location1, conflict.location2, conflict.timeStep);
			constraints.emplace_back(ConstraintType::EDGE, conflict.droneId2,
				conflict.location2, conflict.location1, conflict.timeStep);
			break;

		case ConflictType::FOLLOWING:
			// �����ͻ�����Ӱ�ȫ����Լ��
			constraints.emplace_back(ConstraintType::VERTEX, conflict.droneId1,
				conflict.location1, conflict.timeStep);
			constraints.emplace_back(ConstraintType::VERTEX, conflict.droneId2,
				conflict.location2, conflict.timeStep);
			break;

		default:
			break;
		}

		// ���ʹ����Լ���������Լ��
		if (usePositive && !constraints.empty()) {
			// ��ʵ�֣�Ϊ��һ�����˻������Լ��
			constraints.clear();
			constraints.emplace_back(ConstraintType::POSITIVE, conflict.droneId1,
				conflict.location1, conflict.timeStep);
			constraints.emplace_back(ConstraintType::VERTEX, conflict.droneId2,
				conflict.location1, conflict.timeStep);
		}

		return constraints;
	}

	void CBSPlanner::applyConstraints(int droneId, const std::vector<CBSConstraint>& constraints) {
		// ������Ҫ�޸�SingleDronePlanner��֧��CBSԼ��
		// ��ʱʹ�ü�ʵ��
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

		// �򵥲��ԣ�ѡ�����緢���ĳ�ͻ
		auto earliestConflict = *std::min_element(conflicts.begin(), conflicts.end(),
			[](const Conflict& a, const Conflict& b) {
				return a.timeStep < b.timeStep;
			});

		return earliestConflict;
	}

	bool CBSPlanner::isCardinalConflict(const Conflict& conflict, const Paths& solution) {
		// ��ʵ�֣�����ͻ�Ƿ��ǹؼ���
		// ��ʵʵ����Ҫ������Լ����·�������Ƿ�����
		return true;
	}

	std::vector<CBSNodePtr> CBSPlanner::disjointSplitting(const Conflict& conflict, CBSNodePtr parent) {
		// ����ָ�ļ�ʵ��
		// ��ʵʵ����Ҫ�����ӵ��߼�
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
		// ��Լ����ʵ��
		// ����Ҫ�����ӵ��߼���ȷ��·�����뾭���ض�λ��
	}

	Paths CBSPlanner::getBestSolution() const {
		if (!closedList_.empty()) {
			// �������һ������õģ���
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
			std::cout << "CBS: û�пɱ���Ľ�" << std::endl;
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

		// ���CBSͳ����Ϣ
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

		std::cout << "CBS: ·�������ѱ��浽 " << filename << std::endl;
	}

	void CBSPlanner::printStatistics() const {
		std::cout << "\n=== CBS�㷨ͳ����Ϣ ===" << std::endl;
		std::cout << "���˻�����: " << drones_.size() << std::endl;
		std::cout << "�߲���չ����: " << highLevelExpansions_ << std::endl;
		std::cout << "�Ͳ���չ����: " << lowLevelExpansions_ << std::endl;
		std::cout << "�ܹ滮ʱ��: " << totalPlanningTime_ << " ��" << std::endl;

		auto solution = getBestSolution();
		if (!solution.empty()) {
			double totalCost = 0.0;
			for (size_t i = 0; i < solution.size(); i++) {
				if (!solution[i].empty()) {
					double pathCost = solution[i].back()->gCost;
					totalCost += pathCost;
					std::cout << "���˻� " << (i < drones_.size() ? drones_[i].id : static_cast<int>(i))
						<< " ·������: " << solution[i].size()
						<< ", ����: " << pathCost << std::endl;
				}
			}
			std::cout << "�ܴ���: " << totalCost << std::endl;

			auto conflicts = getCurrentConflicts();
			std::cout << "ʣ���ͻ��: " << conflicts.size() << std::endl;
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
		// �򻯵�����ʽ����ͻ����
		return static_cast<double>(node->conflicts.size());
	}

	void CBSPlanner::prioritizeConflicts(std::vector<Conflict>& conflicts, const Paths& solution) {
		// ��ʱ�䲽�����ͻ
		std::sort(conflicts.begin(), conflicts.end(),
			[](const Conflict& a, const Conflict& b) {
				return a.timeStep < b.timeStep;
			});
	}

	// CBSConstraintManagerʵ��
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