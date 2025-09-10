// Map3DManager.cpp
#include "Map3DManager.h"
#include <iostream>
#include <libpq-fe.h>

Map3DManager& Map3DManager::getInstance() {
	static Map3DManager instance;
	return instance;
}

void Map3DManager::loadFromDatabase(PGconn* conn, const std::string& query) {
	auto start0 = std::chrono::high_resolution_clock::now();
	PGresult* res = PQexec(conn, query.c_str());

	if (PQresultStatus(res) != PGRES_TUPLES_OK) {
		throw std::runtime_error("Query failed");
	}
	auto end0 = std::chrono::high_resolution_clock::now();

	auto duration0 = std::chrono::duration_cast<std::chrono::milliseconds>(end0 - start0);
	std::cout << "数据库查询处理耗时: " << duration0.count() << " 毫秒" << std::endl;
	auto start1 = std::chrono::high_resolution_clock::now();
	auto loadedMap = Map3D::loadFromPostgresfix(res, 8);

	auto end1 = std::chrono::high_resolution_clock::now();

	auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);

	std::cout << "三维数组抽象化处理耗时: " << duration1.count() << " 毫秒" << std::endl;
	if (!loadedMap) {
		throw std::runtime_error("Failed to load map");
	}

	std::lock_guard<std::mutex> lock(map_mutex);
	map = std::move(loadedMap); // Use std::move to transfer ownership of the unique_ptr  
	PQclear(res);
}

std::shared_ptr<Map3D> Map3DManager::getMap() {
	std::lock_guard<std::mutex> lock(map_mutex);
	return map;
}
