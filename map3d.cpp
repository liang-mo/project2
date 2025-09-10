#include "map3d.h"
#include <iostream>
#include <fstream>
#include <set>
#include <cmath>
#include <stdexcept>
#include <regex>
#include <unordered_set>
#include <chrono>

using namespace std;

namespace DronePathfinding {

	Map3D::Map3D(int height, int width, int length)
		: height_(height), width_(width), length_(length) {
		data_.resize(height_, std::vector<std::vector<int>>(width_, std::vector<int>(length_, 0)));
	}

	bool Map3D::isValidIndex(int z, int x, int y) const {
		return z >= 0 && z < height_ && x >= 0 && x < width_ && y >= 0 && y < length_;
	}

	int& Map3D::operator()(int z, int x, int y) {
		if (!isValidIndex(z, x, y)) {
			throw std::out_of_range("Map3D index out of range");
		}
		return data_[z][x][y];
	}

	const int& Map3D::operator()(int z, int x, int y) const {
		if (!isValidIndex(z, x, y)) {
			throw std::out_of_range("Map3D index out of range");
		}
		return data_[z][x][y];
	}

	bool Map3D::hasCoordMapping(int z, int x, int y) const {
		auto key = std::make_tuple(z, x, y);
		return coordMap_.find(key) != coordMap_.end();
	}

	std::tuple<double, double, double> Map3D::getOriginalCoord(int z, int x, int y) const {
		auto key = std::make_tuple(z, x, y);
		auto it = coordMap_.find(key);
		if (it != coordMap_.end()) {
			return it->second;
		}
		throw std::runtime_error("No coordinate mapping found for the given indices");
	}

	void Map3D::setCoordMapping(int z, int x, int y, double origX, double origY, double origZ) {
		auto key = std::make_tuple(z, x, y);
		coordMap_[key] = std::make_tuple(origX, origY, origZ);
	}

	std::unique_ptr<Map3D> Map3D::loadFromJson(const std::string& jsonFile, int precision) {
		std::ifstream file(jsonFile);
		if (!file.is_open()) {
			throw std::runtime_error("Cannot open file: " + jsonFile);
		}

		nlohmann::json data;
		file >> data;

		auto allPoints = data["data"];
		std::set<double> uniqueX, uniqueY, uniqueZ;

		// Extract and process all coordinates
		for (const auto& point : allPoints) {
			double x = std::round(point["X"].get<double>() * std::pow(10, precision)) / std::pow(10, precision);
			double y = std::round(point["Y"].get<double>() * std::pow(10, precision)) / std::pow(10, precision);
			double z = std::round(point["Z"].get<double>() * std::pow(10, precision)) / std::pow(10, precision);

			uniqueX.insert(x);
			uniqueY.insert(y);
			uniqueZ.insert(z);
		}

		// Create sorted coordinate lists
		std::vector<double> xList(uniqueX.begin(), uniqueX.end());
		std::vector<double> yList(uniqueY.begin(), uniqueY.end());
		std::vector<double> zList(uniqueZ.begin(), uniqueZ.end());

		// Create index mappings
		std::unordered_map<double, int> xToRow, yToCol, zToLayer;
		for (size_t i = 0; i < xList.size(); i++) xToRow[xList[i]] = static_cast<int>(i);
		for (size_t i = 0; i < yList.size(); i++) yToCol[yList[i]] = static_cast<int>(i);
		for (size_t i = 0; i < zList.size(); i++) zToLayer[zList[i]] = static_cast<int>(i);

		// Create map object
		int layers = static_cast<int>(zList.size());
		int rows = static_cast<int>(xList.size());
		int cols = static_cast<int>(yList.size());
		auto map = std::make_unique<Map3D>(layers, rows, cols);

		// Save coordinate lists
		map->xList_ = std::move(xList);
		map->yList_ = std::move(yList);
		map->zList_ = std::move(zList);

		// Fill map data
		int mappedCount = 0;
		for (const auto& point : allPoints) {
			try {
				double x = std::round(point["X"].get<double>() * std::pow(10, precision)) / std::pow(10, precision);
				double y = std::round(point["Y"].get<double>() * std::pow(10, precision)) / std::pow(10, precision);
				double z = std::round(point["Z"].get<double>() * std::pow(10, precision)) / std::pow(10, precision);
				int attr = point.value("Attribute", 0);

				if (xToRow.find(x) == xToRow.end() || yToCol.find(y) == yToCol.end() ||
					zToLayer.find(z) == zToLayer.end()) {
					continue;
				}

				int k = zToLayer[z];
				int i = xToRow[x];
				int j = yToCol[y];

				(*map)(k, i, j) = attr;
				map->setCoordMapping(k, i, j, x, y, z);
				mappedCount++;

			}
			catch (const std::exception& e) {
				std::cout << "Warning: Skip invalid point, error: " << e.what() << std::endl;
				continue;
			}
		}

		std::cout << "Successfully mapped grid count: " << mappedCount << std::endl;
		std::cout << "Unmapped grid count: " << (layers * rows * cols - mappedCount) << std::endl;

		return map;
	}

	std::unique_ptr<Map3D> Map3D::loadFromPostgres(PGresult* res, int precision) {
		int result_size = PQntuples(res);
		if (result_size == 0) {
			throw std::runtime_error("No data returned from Postgres.");
		}

		std::set<double> uniqueX, uniqueY, uniqueZ;
		std::regex pattern(R"(POINTZ\(([-\d.]+) ([-\d.]+) ([-\d.]+)\))");

		// 收集所有唯一坐标
		for (int i = 0; i < result_size; ++i) {
			std::string pointStr = PQgetvalue(res, i, 1); // pointt 列
			std::smatch match;

			if (std::regex_match(pointStr, match, pattern)) {
				double x = std::round(std::stod(match[1]) * std::pow(10, precision)) / std::pow(10, precision);
				double y = std::round(std::stod(match[2]) * std::pow(10, precision)) / std::pow(10, precision);
				double z = std::round(std::stod(match[3]) * std::pow(10, precision)) / std::pow(10, precision);

				uniqueX.insert(x);
				uniqueY.insert(y);
				uniqueZ.insert(z);
			}
		}

		// 构建索引映射
		std::vector<double> xList(uniqueX.begin(), uniqueX.end());
		std::vector<double> yList(uniqueY.begin(), uniqueY.end());
		std::vector<double> zList(uniqueZ.begin(), uniqueZ.end());

		std::unordered_map<double, int> xToRow, yToCol, zToLayer;
		for (size_t i = 0; i < xList.size(); i++) xToRow[xList[i]] = static_cast<int>(i);
		for (size_t i = 0; i < yList.size(); i++) yToCol[yList[i]] = static_cast<int>(i);
		for (size_t i = 0; i < zList.size(); i++) zToLayer[zList[i]] = static_cast<int>(i);

		// 创建 map3D
		int layers = static_cast<int>(zList.size());
		int rows = static_cast<int>(xList.size());
		int cols = static_cast<int>(yList.size());


		std::cout << "layers: " << layers << " rows: " << rows << " cols: " << cols << std::endl;
		auto map = std::make_unique<Map3D>(layers, rows, cols);
		map->xList_ = std::move(xList);
		map->yList_ = std::move(yList);
		map->zList_ = std::move(zList);

		int mappedCount = 0;

		// 填充 map3D 数据
		for (int i = 0; i < PQntuples(res); ++i) {
			std::string pointStr = PQgetvalue(res, i, 1);  // pointt
			std::string stateStr = PQgetvalue(res, i, 5);  // state

			std::smatch match;
			if (!std::regex_match(pointStr, match, pattern)) continue;

			double x = std::round(std::stod(match[1]) * std::pow(10, precision)) / std::pow(10, precision);
			double y = std::round(std::stod(match[2]) * std::pow(10, precision)) / std::pow(10, precision);
			double z = std::round(std::stod(match[3]) * std::pow(10, precision)) / std::pow(10, precision);

			if (xToRow.find(x) == xToRow.end() || yToCol.find(y) == yToCol.end() || zToLayer.find(z) == zToLayer.end()) {
				continue;
			}

			int irow = xToRow[x];
			int jcol = yToCol[y];
			int klayer = zToLayer[z];

			// state 为 true（"t"）为障碍物；false（"f"）为可通行
			bool isObstacle = (stateStr == "t");
			int attr = isObstacle ? 1 : 0;

			(*map)(klayer, irow, jcol) = attr;
			map->setCoordMapping(klayer, irow, jcol, x, y, z);
			mappedCount++;
		}

		std::cout << "Mapped " << mappedCount << " points from Postgres result.\n";
		std::cout << "Unmapped grid cells: " << (layers * rows * cols - mappedCount) << std::endl;

		return map;
	}
	std::tuple<int, int, int> Map3D::getIndexFromCoordinate(double x, double y, double z, int precision) const {
		auto findNearestIndex = [&](const std::vector<double>& vec, double value) -> int {
			if (vec.empty()) throw std::out_of_range("Coordinate list empty.");
			// 使用lower_bound找到大致位置
			auto it = std::lower_bound(vec.begin(), vec.end(), value);
			if (it == vec.end()) return static_cast<int>(vec.size()) - 1;
			if (it == vec.begin()) return 0;

			auto prev = it - 1;
			// 比较前后哪个更近
			if (std::abs(*prev - value) < std::abs(*it - value))
				return static_cast<int>(prev - vec.begin());
			else
				return static_cast<int>(it - vec.begin());
			};

		int row = findNearestIndex(xList_, x);
		int col = findNearestIndex(yList_, y);
		int layer = findNearestIndex(zList_, z);

		return { row, col, layer };
	}



	std::unique_ptr<Map3D> Map3D::loadFromPostgresfix(PGresult* res, int precision) {
		auto totalStart = std::chrono::high_resolution_clock::now();

		int result_size = PQntuples(res);
		if (result_size == 0) {
			throw std::runtime_error("No data returned from Postgres.");
		}

		const int64_t scale = static_cast<int64_t>(std::pow(10, precision));
		std::unordered_set<int64_t> uniqueXInt, uniqueYInt, uniqueZInt;
		std::unordered_map<int64_t, double> intToX, intToY, intToZ;
		std::vector<std::tuple<int64_t, int64_t, int64_t, bool, double, double, double>> parsedData;
		parsedData.reserve(result_size);

		// Part 1: 解析 PGresult -> parsedData
		auto start0 = std::chrono::high_resolution_clock::now();
		for (int i = 0; i < result_size; ++i) {
			const char* pointStr = PQgetvalue(res, i, 0); // pointt 列
			const char* stateStr = PQgetvalue(res, i, 2); // state 列

			double x, y, z;
			if (sscanf_s(pointStr, "POINTZ(%lf %lf %lf)", &x, &y, &z) != 3) {
				continue;
			}

			int64_t xi = static_cast<int64_t>(std::round(x * scale));
			int64_t yi = static_cast<int64_t>(std::round(y * scale));
			int64_t zi = static_cast<int64_t>(std::round(z * scale));

			uniqueXInt.insert(xi);
			uniqueYInt.insert(yi);
			uniqueZInt.insert(zi);

			intToX[xi] = x;
			intToY[yi] = y;
			intToZ[zi] = z;

			bool isObstacle = (stateStr[0] == 't'); // "t" or "f"
			parsedData.emplace_back(xi, yi, zi, isObstacle, x, y, z);
		}
		auto end0 = std::chrono::high_resolution_clock::now();
		std::cout << "Part 1 (解析+离散化) 耗时: "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(end0 - start0).count()
			<< " 毫秒" << std::endl;

		// Part 2: 构建索引、坐标列表
		auto start1 = std::chrono::high_resolution_clock::now();
		std::vector<int64_t> xIntList(uniqueXInt.begin(), uniqueXInt.end());
		std::vector<int64_t> yIntList(uniqueYInt.begin(), uniqueYInt.end());
		std::vector<int64_t> zIntList(uniqueZInt.begin(), uniqueZInt.end());

		std::sort(xIntList.begin(), xIntList.end());
		std::sort(yIntList.begin(), yIntList.end());
		std::sort(zIntList.begin(), zIntList.end());

		std::vector<double> xList, yList, zList;
		std::unordered_map<int64_t, int> xToRow, yToCol, zToLayer;

		for (size_t i = 0; i < xIntList.size(); ++i) {
			xToRow[xIntList[i]] = static_cast<int>(i);
			xList.push_back(intToX[xIntList[i]]);
		}
		for (size_t i = 0; i < yIntList.size(); ++i) {
			yToCol[yIntList[i]] = static_cast<int>(i);
			yList.push_back(intToY[yIntList[i]]);
		}
		for (size_t i = 0; i < zIntList.size(); ++i) {
			zToLayer[zIntList[i]] = static_cast<int>(i);
			zList.push_back(intToZ[zIntList[i]]);
		}
		auto end1 = std::chrono::high_resolution_clock::now();
		std::cout << "Part 2 (坐标索引映射与排序) 耗时: "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1).count()
			<< " 毫秒" << std::endl;

		// Part 3: 初始化 Map3D 对象
		auto start2 = std::chrono::high_resolution_clock::now();
		int layers = static_cast<int>(zList.size());
		int rows = static_cast<int>(xList.size());
		int cols = static_cast<int>(yList.size());

		std::cout << "layers: " << layers << " rows: " << rows << " cols: " << cols << std::endl;

		auto map = std::make_unique<Map3D>(layers, rows, cols);
		map->xList_ = std::move(xList);
		map->yList_ = std::move(yList);
		map->zList_ = std::move(zList);
		auto end2 = std::chrono::high_resolution_clock::now();
		std::cout << "Part 3 (Map3D 对象初始化) 耗时: "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2).count()
			<< " 毫秒" << std::endl;

		// Part 4: 映射数据写入网格
		auto start3 = std::chrono::high_resolution_clock::now();
		int mappedCount = 0;
		for (const auto& [xi, yi, zi, isObstacle, x, y, z] : parsedData) {
			auto xIt = xToRow.find(xi);
			auto yIt = yToCol.find(yi);
			auto zIt = zToLayer.find(zi);

			if (xIt == xToRow.end() || yIt == yToCol.end() || zIt == zToLayer.end()) continue;

			int irow = xIt->second;
			int jcol = yIt->second;
			int klayer = zIt->second;

			int attr = isObstacle ? 1 : 0;
			(*map)(klayer, irow, jcol) = attr;
			map->setCoordMapping(klayer, irow, jcol, x, y, z);
			mappedCount++;
		}
		auto end3 = std::chrono::high_resolution_clock::now();
		std::cout << "Part 4 (网格赋值与坐标映射) 耗时: "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(end3 - start3).count()
			<< " 毫秒" << std::endl;

		std::cout << "Mapped " << mappedCount << " points from Postgres result.\n";
		std::cout << "Unmapped grid cells: " << (layers * rows * cols - mappedCount) << std::endl;

		auto totalEnd = std::chrono::high_resolution_clock::now();
		std::cout << "总耗时: "
			<< std::chrono::duration_cast<std::chrono::milliseconds>(totalEnd - totalStart).count()
			<< " 毫秒" << std::endl;

		return map;
	}




	void Map3D::printStatistics() const {
		std::cout << "Map statistics:" << std::endl;
		std::cout << "  Dimensions: " << width_ << " x " << length_ << " x " << height_ << std::endl;
		std::cout << "  Total grids: " << (width_ * length_ * height_) << std::endl;

		int obstacleCount = 0;
		for (int z = 0; z < height_; z++) {
			for (int x = 0; x < width_; x++) {
				for (int y = 0; y < length_; y++) {
					if (data_[z][x][y] != 0) {
						obstacleCount++;
					}
				}
			}
		}

		std::cout << "  Obstacle count: " << obstacleCount << std::endl;
		std::cout << "  Passable rate: " << (100.0 * (width_ * length_ * height_ - obstacleCount) / (width_ * length_ * height_)) << "%" << std::endl;
	}

	std::vector<Point3D> Map3D::getObstacles() const {
		std::vector<Point3D> obstacles;
		for (int z = 0; z < height_; z++) {
			for (int x = 0; x < width_; x++) {
				for (int y = 0; y < length_; y++) {
					if (data_[z][x][y] != 0) {
						obstacles.emplace_back(x, y, z);
					}
				}
			}
		}
		return obstacles;
	}

} // namespace DronePathfinding
