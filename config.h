#pragma once


namespace DronePathfinding {

	// 算法配置参数
	struct AlgorithmConfig {
		// A*算法权重
		double weightG = 1.0;           // g值权重
		double weightH = 1.0;           // h值权重
		double weightSafety = 50;      // 安全距离惩罚权重

		// 移动权重
		double weightStraight = 1;    // 直飞
		double weightVertical = 1.0;    // 上下飞
		double weightHorizontal = 1.0;  // 左右飞
		double weightDiagonal = 1.5;    // 斜上下飞

		// 安全距离参数
		int preferredSafetyDistance = 5;  // 首选安全距离
		int minSafetyDistance = 1;        // 最小安全距离

		// 多无人机参数
		int maxIterations = 900;         // 最大迭代次数
		double conflictWeight = 10.0;     // 冲突惩罚权重
		int timeHorizon = 100;            // 时间窗口

		// 性能参数
		int maxNodesExplored = 100000;    // 最大探索节点数
		double timeLimit = 30.0;          // 时间限制（秒）

		// 地图参数
		int passableTag = 0;              // 可通行标记
	};

	// 全局配置实例
	extern AlgorithmConfig g_config;

} // namespace DronePathfinding