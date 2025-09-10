#pragma once

#include "types.h"
#include <vector>
#include <string>
#include <nlohmann/json.hpp>
#include <libpq-fe.h>


namespace DronePathfinding {

	class Map3D {
	private:
		int height_, width_, length_;  // Z, X, Y 维度
		std::vector<std::vector<std::vector<int>>> data_;
		std::unordered_map<std::tuple<int, int, int>, std::tuple<double, double, double>, TupleHash> coordMap_;
		std::vector<double> xList_, yList_, zList_;

	public:
		Map3D(int height, int width, int length);

		// 基础访问方法
		bool isValidIndex(int z, int x, int y) const;
		int& operator()(int z, int x, int y);
		const int& operator()(int z, int x, int y) const;

		// 维度访问
		int getHeight() const { return height_; }
		int getWidth() const { return width_; }
		int getLength() const { return length_; }

		// 坐标转换
		bool hasCoordMapping(int z, int x, int y) const;
		std::tuple<double, double, double> getOriginalCoord(int z, int x, int y) const;

		// 从JSON加载地图
		static std::unique_ptr<Map3D> loadFromJson(const std::string& jsonFile, int precision = 8);
		// 从Postgres加载地图
		static std::unique_ptr<Map3D> loadFromPostgres(PGresult* res, int precision = 8);
		static std::unique_ptr<Map3D> loadFromPostgresfix(PGresult* res, int precision = 8);

		std::tuple<int, int, int> getIndexFromCoordinate(double x, double y, double z, int precision = 6) const;

		// 地图分析
		void printStatistics() const;
		std::vector<Point3D> getObstacles() const;

	private:
		void setCoordMapping(int z, int x, int y, double origX, double origY, double origZ);
	};

} // namespace DronePathfinding