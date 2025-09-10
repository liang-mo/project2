#pragma once

#include "types.h"
#include "single_drone_planner.h"
#include "conflict_detector.h"
#include <queue>
#include <memory>


namespace DronePathfinding {

	// CBSԼ������
	enum class ConstraintType {
		VERTEX,    // ����Լ������ֹ���ض�ʱ������ض�λ��
		EDGE,      // ��Լ������ֹ���ض�ʱ��ִ���ض��ƶ�
		POSITIVE   // ��Լ�����������ض�ʱ������ض�λ��
	};

	// CBSԼ���ṹ
	struct CBSConstraint {
		ConstraintType type;
		int droneId;
		Point3D location;
		Point3D fromLocation;  // ���ڱ�Լ��
		int timeStep;

		CBSConstraint(ConstraintType t, int id, const Point3D& loc, int time)
			: type(t), droneId(id), location(loc), timeStep(time) {
		}

		CBSConstraint(ConstraintType t, int id, const Point3D& from, const Point3D& to, int time)
			: type(t), droneId(id), location(to), fromLocation(from), timeStep(time) {
		}
	};

	// CBS�ڵ㣨�������еĽڵ㣩
	struct CBSNode {
		std::vector<CBSConstraint> constraints;  // Լ������
		Paths solution;                          // ��ǰ�⣨�������˻�·����
		std::vector<Conflict> conflicts;         // ��ǰ��ĳ�ͻ
		double cost;                            // ����ܴ���
		int depth;                              // �������

		CBSNode() : cost(0.0), depth(0) {}

		// �������ܴ���
		void calculateCost() {
			cost = 0.0;
			for (const auto& path : solution) {
				if (!path.empty()) {
					cost += path.back()->gCost;  // ʹ��·��������Ϊ����
				}
			}
		}

		// �ȽϺ������������ȶ��У�
		bool operator>(const CBSNode& other) const {
			if (std::abs(cost - other.cost) < 1e-6) {
				return conflicts.size() > other.conflicts.size();  // ��ͻ�ٵ�����
			}
			return cost > other.cost;
		}
	};

	using CBSNodePtr = std::shared_ptr<CBSNode>;

	// CBS�滮��
	class CBSPlanner {
	private:
		const Map3D& map_;
		std::vector<DroneInfo> drones_;
		std::unique_ptr<SingleDronePlanner> lowLevelPlanner_;

		// CBS�������
		std::priority_queue<CBSNodePtr, std::vector<CBSNodePtr>,
			std::function<bool(const CBSNodePtr&, const CBSNodePtr&)>> openList_;
		std::vector<CBSNodePtr> closedList_;

		// ͳ����Ϣ
		int highLevelExpansions_;
		int lowLevelExpansions_;
		double totalPlanningTime_;

		// CBS����
		int maxHighLevelNodes_;
		int maxLowLevelNodes_;
		double timeLimit_;
		bool useDisjointSplitting_;  // �Ƿ�ʹ�÷���ָ�
		bool usePositiveConstraints_; // �Ƿ�ʹ����Լ��

	public:
		explicit CBSPlanner(const Map3D& map);
		~CBSPlanner() = default;

		// ���ò���
		void setMaxHighLevelNodes(int maxNodes) { maxHighLevelNodes_ = maxNodes; }
		void setMaxLowLevelNodes(int maxNodes) { maxLowLevelNodes_ = maxNodes; }
		void setTimeLimit(double timeLimit) { timeLimit_ = timeLimit; }
		void setUseDisjointSplitting(bool use) { useDisjointSplitting_ = use; }
		void setUsePositiveConstraints(bool use) { usePositiveConstraints_ = use; }

		// ���˻�����
		void addDrone(const DroneInfo& drone);
		void clearDrones();
		const std::vector<DroneInfo>& getDrones() const { return drones_; }

		// ��Ҫ�滮�ӿ�
		bool planPaths();

		// �������
		Paths getBestSolution() const;
		std::vector<Conflict> getCurrentConflicts() const;

		// ͳ����Ϣ
		int getHighLevelExpansions() const { return highLevelExpansions_; }
		int getLowLevelExpansions() const { return lowLevelExpansions_; }
		double getTotalPlanningTime() const { return totalPlanningTime_; }

		// ������
		void savePathsToJson(const std::string& filename) const;
		void printStatistics() const;

	private:
		// CBS�����㷨
		CBSNodePtr createRootNode();
		std::vector<CBSNodePtr> expandNode(CBSNodePtr node);
		bool solveLowLevel(CBSNodePtr node);

		// Լ������
		std::vector<CBSConstraint> generateConstraints(const Conflict& conflict, bool usePositive = false);
		void applyConstraints(int droneId, const std::vector<CBSConstraint>& constraints);
		void clearConstraints();

		// ��ͻ����
		Conflict selectConflict(const std::vector<Conflict>& conflicts);
		bool isCardinalConflict(const Conflict& conflict, const Paths& solution);

		// �Ż�����
		std::vector<CBSNodePtr> disjointSplitting(const Conflict& conflict, CBSNodePtr parent);
		void addPositiveConstraints(CBSNodePtr node, const Conflict& conflict);

		// ���߷���
		bool isValidSolution(const Paths& solution) const;
		void reset();

		// ����ʽ�Ż�
		double calculateHeuristic(CBSNodePtr node);
		void prioritizeConflicts(std::vector<Conflict>& conflicts, const Paths& solution);
	};

	// CBSԼ�������������ڵͲ�滮����
	class CBSConstraintManager {
	private:
		std::vector<CBSConstraint> constraints_;
		int droneId_;

	public:
		explicit CBSConstraintManager(int droneId) : droneId_(droneId) {}

		void addConstraint(const CBSConstraint& constraint);
		void clearConstraints();

		bool isConstraintViolated(const Point3D& location, int timeStep) const;
		bool isConstraintViolated(const Point3D& from, const Point3D& to, int timeStep) const;
		bool hasPositiveConstraint(const Point3D& location, int timeStep) const;

		const std::vector<CBSConstraint>& getConstraints() const { return constraints_; }
	};

} // namespace DronePathfinding