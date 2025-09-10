#pragma once

#include "types.h"
#include "map3d.h"
#include "single_drone_planner.h"
#include "conflict_detector.h"
#include <vector>
#include <memory>


namespace DronePathfinding {

	class MultiDronePlanner {
	private:
		const Map3D& map_;
		std::vector<DroneInfo> drones_;
		std::unique_ptr<SingleDronePlanner> singlePlanner_;

		// �滮���
		Paths currentPaths_;
		std::vector<Conflict> currentConflicts_;

		// ͳ����Ϣ
		int iterationCount_;
		int totalNodesExplored_;
		double totalPlanningTime_;
		std::string lastErrorMessage_;
	public:
		std::string getLastError() const { return lastErrorMessage_; }
		explicit MultiDronePlanner(const Map3D& map);
		~MultiDronePlanner() = default;

		// ���˻�����
		void addDrone(const DroneInfo& drone);
		void clearDrones();
		const std::vector<DroneInfo>& getDrones() const { return drones_; }

		// ·���滮
		bool planPaths();

		// �������
		const Paths& getPaths() const { return currentPaths_; }
		const std::vector<Conflict>& getConflicts() const { return currentConflicts_; }

		// ͳ����Ϣ
		int getIterationCount() const { return iterationCount_; }
		int getTotalNodesExplored() const { return totalNodesExplored_; }
		double getTotalPlanningTime() const { return totalPlanningTime_; }

		// ������
		void savePathsToJson(const std::string& filename) const;
		void printStatistics() const;

		nlohmann::json getPathsAsJson() const {
			nlohmann::json output;
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

			output["statistics"] = {
				{"total_drones", drones_.size()},
				{"total_conflicts", currentConflicts_.size()},
				{"iteration_count", iterationCount_},
				{"planning_time", totalPlanningTime_}
			};

			return output;
		}


	private:
		// �����㷨
		bool planWithPriorityBasedApproach();
		bool planWithIterativeApproach();

		// ��ͻ���
		bool resolveConflicts();
		void updateReservations(const Paths& paths, int excludeDroneId = -1);

		// ���߷���
		void sortDronesByPriority();
		std::unordered_set<SpaceTimePoint, SpaceTimePointHash>
			pathsToSpaceTimeReservations(const Paths& paths, int excludeDroneId = -1) const;

		// ��֤����
		bool validatePaths(const Paths& paths) const;
		void reset();
	};

} // namespace DronePathfinding