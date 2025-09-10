#pragma once


namespace DronePathfinding {

	// �㷨���ò���
	struct AlgorithmConfig {
		// A*�㷨Ȩ��
		double weightG = 1.0;           // gֵȨ��
		double weightH = 1.0;           // hֵȨ��
		double weightSafety = 50;      // ��ȫ����ͷ�Ȩ��

		// �ƶ�Ȩ��
		double weightStraight = 1;    // ֱ��
		double weightVertical = 1.0;    // ���·�
		double weightHorizontal = 1.0;  // ���ҷ�
		double weightDiagonal = 1.5;    // б���·�

		// ��ȫ�������
		int preferredSafetyDistance = 5;  // ��ѡ��ȫ����
		int minSafetyDistance = 1;        // ��С��ȫ����

		// �����˻�����
		int maxIterations = 900;         // ����������
		double conflictWeight = 10.0;     // ��ͻ�ͷ�Ȩ��
		int timeHorizon = 100;            // ʱ�䴰��

		// ���ܲ���
		int maxNodesExplored = 100000;    // ���̽���ڵ���
		double timeLimit = 30.0;          // ʱ�����ƣ��룩

		// ��ͼ����
		int passableTag = 0;              // ��ͨ�б��
	};

	// ȫ������ʵ��
	extern AlgorithmConfig g_config;

} // namespace DronePathfinding