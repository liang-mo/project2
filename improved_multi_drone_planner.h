#pragma once

#include "types.h"
#include "map3d.h"
#include "single_drone_planner.h"
#include "conflict_detector.h"
#include <vector>
#include <memory>
#include <unordered_set>


namespace DronePathfinding {

	// �Ľ��ĳ�ͻ�������
	enum class ConflictResolutionStrategy {
		PRIORITY_BASED,     // �������ȼ�
		TIME_DELAY,         // ʱ���ӳ�
		SPATIAL_SEPARATION, // �ռ����
		ADAPTIVE_SAFETY     // ����Ӧ��ȫ����
	};

	// ·����������ָ��
	struct PathQualityMetrics {
		double totalLength;
		double averageSafety;
		int conflictCount;
		double smoothness;
		double efficiency;

		double getOverallScore() const {
			return efficiency * 0.4 + (1.0 / (conflictCount + 1)) * 0.3 +
				averageSafety * 0.2 + smoothness * 0.1;
		}
	};

	// �Ľ��Ķ����˻��滮��
	class ImprovedMultiDronePlanner {
	private:
		const Map3D& map_;
		std::vector<DroneInfo> drones_;
		std::unique_ptr<SingleDronePlanner> singlePlanner_;

		// �滮���
		Paths currentPaths_;
		std::vector<Conflict> currentConflicts_;
		PathQualityMetrics qualityMetrics_;

		// ͳ����Ϣ
		int iterationCount_;
		int totalNodesExplored_;
		double totalPlanningTime_;

		// �Ľ�����
		ConflictResolutionStrategy strategy_;
		bool useAdaptiveSafety_;
		bool usePathSmoothing_;
		bool useConflictPrediction_;
		double qualityThreshold_;

	public:
		explicit ImprovedMultiDronePlanner(const Map3D& map);
		~ImprovedMultiDronePlanner() = default;

		// ���øĽ�����
		void setConflictResolutionStrategy(ConflictResolutionStrategy strategy) { strategy_ = strategy; }
		void setUseAdaptiveSafety(bool use) { useAdaptiveSafety_ = use; }
		void setUsePathSmoothing(bool use) { usePathSmoothing_ = use; }
		void setUseConflictPrediction(bool use) { useConflictPrediction_ = use; }
		void setQualityThreshold(double threshold) { qualityThreshold_ = threshold; }

		// ���˻�����
		void addDrone(const DroneInfo& drone);
		void clearDrones();
		const std::vector<DroneInfo>& getDrones() const { return drones_; }

		// ·���滮
		bool planPaths();

		// �������
		const Paths& getPaths() const { return currentPaths_; }
		const std::vector<Conflict>& getConflicts() const { return currentConflicts_; }
		const PathQualityMetrics& getQualityMetrics() const { return qualityMetrics_; }

		// ͳ����Ϣ
		int getIterationCount() const { return iterationCount_; }
		int getTotalNodesExplored() const { return totalNodesExplored_; }
		double getTotalPlanningTime() const { return totalPlanningTime_; }
		bool isDirectPathClear(const Point3D& start, const Point3D& end) const;


		// ������
		void savePathsToJson(const std::string& filename) const;
		void printStatistics() const;

	private:
		// ���ĸĽ��㷨
		bool planWithImprovedStrategy();
		bool resolveConflictsIntelligently();

		// ��ͻ�������
		bool resolvePriorityBased(const std::vector<Conflict>& conflicts);
		bool resolveTimeDelay(const std::vector<Conflict>& conflicts);
		bool resolveSpatialSeparation(const std::vector<Conflict>& conflicts);
		bool resolveAdaptiveSafety(const std::vector<Conflict>& conflicts);

		// ·���Ż�
		Path smoothPath(const Path& originalPath) const;
		Path optimizePathSafety(const Path& originalPath, int droneId) const;
		std::vector<Point3D> predictConflictZones() const;

		// ����Ӧ��ȫ����
		int calculateAdaptiveSafetyDistance(const Point3D& point, int droneId) const;
		void updateSafetyDistances();

		// ��������
		PathQualityMetrics evaluatePathQuality(const Paths& paths) const;
		double calculatePathSmoothness(const Path& path) const;
		double calculatePathEfficiency(const Path& path, const DroneInfo& drone) const;
		double calculateAverageSafety(const Paths& paths) const;

		// ���߷���
		void sortDronesByPriority();
		void updateReservations(const Paths& paths, int excludeDroneId = -1);
		std::unordered_set<SpaceTimePoint, SpaceTimePointHash>
			pathsToSpaceTimeReservations(const Paths& paths, int excludeDroneId = -1) const;

		// ��֤������
		bool validatePaths(const Paths& paths) const;
		void reset();

		// ��ͻԤ��
		std::vector<Conflict> predictFutureConflicts(const Paths& paths) const;
		bool isHighConflictZone(const Point3D& point) const;
	};

} // namespace DronePathfinding