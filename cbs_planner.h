#pragma once

#include "types.h"
#include "single_drone_planner.h"
#include "conflict_detector.h"
#include <queue>
#include <memory>


namespace DronePathfinding {

	// CBS约束类型
	enum class ConstraintType {
		VERTEX,    // 顶点约束：禁止在特定时间访问特定位置
		EDGE,      // 边约束：禁止在特定时间执行特定移动
		POSITIVE   // 正约束：必须在特定时间访问特定位置
	};

	// CBS约束结构
	struct CBSConstraint {
		ConstraintType type;
		int droneId;
		Point3D location;
		Point3D fromLocation;  // 用于边约束
		int timeStep;

		CBSConstraint(ConstraintType t, int id, const Point3D& loc, int time)
			: type(t), droneId(id), location(loc), timeStep(time) {
		}

		CBSConstraint(ConstraintType t, int id, const Point3D& from, const Point3D& to, int time)
			: type(t), droneId(id), location(to), fromLocation(from), timeStep(time) {
		}
	};

	// CBS节点（搜索树中的节点）
	struct CBSNode {
		std::vector<CBSConstraint> constraints;  // 约束集合
		Paths solution;                          // 当前解（所有无人机路径）
		std::vector<Conflict> conflicts;         // 当前解的冲突
		double cost;                            // 解的总代价
		int depth;                              // 搜索深度

		CBSNode() : cost(0.0), depth(0) {}

		// 计算解的总代价
		void calculateCost() {
			cost = 0.0;
			for (const auto& path : solution) {
				if (!path.empty()) {
					cost += path.back()->gCost;  // 使用路径长度作为代价
				}
			}
		}

		// 比较函数（用于优先队列）
		bool operator>(const CBSNode& other) const {
			if (std::abs(cost - other.cost) < 1e-6) {
				return conflicts.size() > other.conflicts.size();  // 冲突少的优先
			}
			return cost > other.cost;
		}
	};

	using CBSNodePtr = std::shared_ptr<CBSNode>;

	// CBS规划器
	class CBSPlanner {
	private:
		const Map3D& map_;
		std::vector<DroneInfo> drones_;
		std::unique_ptr<SingleDronePlanner> lowLevelPlanner_;

		// CBS搜索相关
		std::priority_queue<CBSNodePtr, std::vector<CBSNodePtr>,
			std::function<bool(const CBSNodePtr&, const CBSNodePtr&)>> openList_;
		std::vector<CBSNodePtr> closedList_;

		// 统计信息
		int highLevelExpansions_;
		int lowLevelExpansions_;
		double totalPlanningTime_;

		// CBS参数
		int maxHighLevelNodes_;
		int maxLowLevelNodes_;
		double timeLimit_;
		bool useDisjointSplitting_;  // 是否使用分离分割
		bool usePositiveConstraints_; // 是否使用正约束

	public:
		explicit CBSPlanner(const Map3D& map);
		~CBSPlanner() = default;

		// 配置参数
		void setMaxHighLevelNodes(int maxNodes) { maxHighLevelNodes_ = maxNodes; }
		void setMaxLowLevelNodes(int maxNodes) { maxLowLevelNodes_ = maxNodes; }
		void setTimeLimit(double timeLimit) { timeLimit_ = timeLimit; }
		void setUseDisjointSplitting(bool use) { useDisjointSplitting_ = use; }
		void setUsePositiveConstraints(bool use) { usePositiveConstraints_ = use; }

		// 无人机管理
		void addDrone(const DroneInfo& drone);
		void clearDrones();
		const std::vector<DroneInfo>& getDrones() const { return drones_; }

		// 主要规划接口
		bool planPaths();

		// 结果访问
		Paths getBestSolution() const;
		std::vector<Conflict> getCurrentConflicts() const;

		// 统计信息
		int getHighLevelExpansions() const { return highLevelExpansions_; }
		int getLowLevelExpansions() const { return lowLevelExpansions_; }
		double getTotalPlanningTime() const { return totalPlanningTime_; }

		// 结果输出
		void savePathsToJson(const std::string& filename) const;
		void printStatistics() const;

	private:
		// CBS核心算法
		CBSNodePtr createRootNode();
		std::vector<CBSNodePtr> expandNode(CBSNodePtr node);
		bool solveLowLevel(CBSNodePtr node);

		// 约束处理
		std::vector<CBSConstraint> generateConstraints(const Conflict& conflict, bool usePositive = false);
		void applyConstraints(int droneId, const std::vector<CBSConstraint>& constraints);
		void clearConstraints();

		// 冲突分析
		Conflict selectConflict(const std::vector<Conflict>& conflicts);
		bool isCardinalConflict(const Conflict& conflict, const Paths& solution);

		// 优化策略
		std::vector<CBSNodePtr> disjointSplitting(const Conflict& conflict, CBSNodePtr parent);
		void addPositiveConstraints(CBSNodePtr node, const Conflict& conflict);

		// 工具方法
		bool isValidSolution(const Paths& solution) const;
		void reset();

		// 启发式优化
		double calculateHeuristic(CBSNodePtr node);
		void prioritizeConflicts(std::vector<Conflict>& conflicts, const Paths& solution);
	};

	// CBS约束管理器（用于低层规划器）
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